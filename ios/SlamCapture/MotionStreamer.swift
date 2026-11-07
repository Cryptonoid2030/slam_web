import CoreMotion

/// Samples CoreMotion at 100 Hz and hands out batches of
/// `[t, ax, ay, az, gx, gy, gz]` rows: acceleration = userAcceleration + gravity in g,
/// rotation rate in rad/s, all in the device frame, `t` in seconds since boot (the same
/// clock as ARFrame.timestamp). The Mac converts axes and units.
final class MotionStreamer {
    private let manager = CMMotionManager()
    private let queue: OperationQueue = {
        let q = OperationQueue()
        q.name = "motion"
        q.maxConcurrentOperationCount = 1
        q.qualityOfService = .userInteractive
        return q
    }()
    private let lock = NSLock()
    private var rows: [[Double]] = []

    var isAvailable: Bool { manager.isDeviceMotionAvailable }

    func start() {
        guard manager.isDeviceMotionAvailable else { return }
        manager.deviceMotionUpdateInterval = 1.0 / 100.0
        manager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { [weak self] motion, _ in
            guard let self, let m = motion else { return }
            let row = [m.timestamp,
                       m.userAcceleration.x + m.gravity.x,
                       m.userAcceleration.y + m.gravity.y,
                       m.userAcceleration.z + m.gravity.z,
                       m.rotationRate.x, m.rotationRate.y, m.rotationRate.z]
            self.lock.lock()
            self.rows.append(row)
            if self.rows.count > 2000 { self.rows.removeFirst(self.rows.count - 2000) }   // never grow without bound
            self.lock.unlock()
        }
    }

    func stop() {
        manager.stopDeviceMotionUpdates()
        lock.lock(); rows.removeAll(); lock.unlock()
    }

    /// Everything gathered since the last call, as the relay's IMU JSON message (nil when empty).
    func drainMessage() -> String? {
        lock.lock()
        let batch = rows
        rows.removeAll(keepingCapacity: true)
        lock.unlock()
        guard !batch.isEmpty else { return nil }
        let payload: [String: Any] = ["type": "imu", "samples": batch]
        guard let data = try? JSONSerialization.data(withJSONObject: payload) else { return nil }
        return String(data: data, encoding: .utf8)
    }
}
