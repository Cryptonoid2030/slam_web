import ARKit
import CoreImage
import ImageIO
import UIKit

/// Runs ARKit (LiDAR scene depth + camera) and CoreMotion and streams them to the relay.
/// The phone does no SLAM. By default it picks corner landmarks itself (see LandmarkExtractor.swift)
/// and sends only those, so no color image or depth map ever leaves the phone; the page can ask for raw frames instead.
final class CaptureEngine: NSObject, ObservableObject, ARSessionDelegate {
    // Raw camera frames are sent at 4 Hz. The Mac keeps the ones 0.6-0.95 s apart; sending denser
    // than that lets it pick frames without ever exceeding the estimator's 1 s limit.
    // Landmark packets are small, so the page's `config` sets the interval instead (0.7 s by default).
    private let cameraInterval: TimeInterval = 0.25
    /// Wait this long for the page's config before falling back to raw frames (an older page never sends one).
    private let configGrace: TimeInterval = 3
    private let jpegQuality = 0.7

    @Published var status = "Idle"
    @Published var detail = ""
    @Published var running = false
    @Published var sentFrames = 0
    @Published var runnerConnected = false
    @Published var mirrorViews = 0
    @Published var mode = "waiting for the page"
    @Published var lidarSupported = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)

    private let session = ARSession()
    private let relay = RelayClient()
    private let motion = MotionStreamer()
    private let delegateQueue = DispatchQueue(label: "capture.ar.delegate")
    private let sendQueue = DispatchQueue(label: "capture.send", qos: .userInitiated)
    private let ciContext = CIContext()
    private var lastSent: TimeInterval = 0
    private var busy = false          // touched only on delegateQueue
    private var streaming = false     // read on delegateQueue; set when the relay opens or capture stops
    private var imuTimer: DispatchSourceTimer?
    private var relayURL: URL?
    private let configLock = NSLock()
    private var landmarkConfig = LandmarkConfig()
    private var configReceived = false
    private var imuOnly = false
    private var streamStart = Date()
    private var settings: (config: LandmarkConfig, received: Bool, imuOnly: Bool) {
        configLock.lock(); defer { configLock.unlock() }
        return (landmarkConfig, configReceived, imuOnly)
    }

    override init() {
        super.init()
        session.delegate = self
        session.delegateQueue = delegateQueue
        relay.onOpen = { [weak self] in self?.relayOpened() }
        relay.onClose = { [weak self] message in self?.relayClosed(message) }
        relay.onControl = { [weak self] action, reason in self?.relayControl(action, reason) }
        relay.onConfig = { [weak self] json in
            guard let self, let config = LandmarkConfig(json: json) else { return }
            self.configLock.lock()
            self.landmarkConfig = config
            self.configReceived = true
            self.imuOnly = json["source"] as? String == "d435"
            self.configLock.unlock()
            DispatchQueue.main.async { self.mode = json["source"] as? String == "d435" ? "IMU only" : config.landmarks ? "landmarks \(config.width)x\(config.height)" : "raw frames" }
        }
        relay.onPeers = { [weak self] peers in
            DispatchQueue.main.async {
                self?.runnerConnected = peers.runner > 0
                self?.mirrorViews = peers.views
            }
        }
    }

    // MARK: start / stop

    /// `address` is `host:port` (default port 8787). `token` is optional and matches `--token` on the relay.
    func start(address: String, token: String) {
        guard !running else { return }
        guard lidarSupported else { setStatus("This iPhone has no LiDAR scene depth (needs a Pro model)."); return }
        guard motion.isAvailable else { setStatus("Motion sensors are unavailable."); return }
        guard let url = Self.ingestURL(address: address, token: token) else { setStatus("Enter the Mac's address, e.g. 192.168.1.20:8787"); return }
        relayURL = url
        setStatus("Connecting to \(url.host ?? "relay")…")
        relay.connect(url: url)
    }

    func stop(_ message: String = "Stopped") {
        guard running || relayURL != nil else { return }
        streaming = false
        session.pause()
        motion.stop()
        imuTimer?.cancel(); imuTimer = nil
        relayURL = nil          // before disconnect(), so the close callback cannot re-enter stop()
        relay.disconnect()
        DispatchQueue.main.async {
            UIApplication.shared.isIdleTimerDisabled = false
            self.running = false
            self.status = message
            self.runnerConnected = false
        }
    }

    static func ingestURL(address: String, token: String) -> URL? {
        var text = address.trimmingCharacters(in: .whitespaces)
        guard !text.isEmpty else { return nil }
        if text.hasPrefix("http://") { text.removeFirst(7) }
        if text.hasPrefix("ws://") { text.removeFirst(5) }
        if let slash = text.firstIndex(of: "/") { text = String(text[..<slash]) }
        if !text.contains(":") { text += ":8787" }
        var string = "ws://\(text)/ingest"
        if !token.isEmpty, let encoded = token.addingPercentEncoding(withAllowedCharacters: .urlQueryAllowed) {
            string += "?token=\(encoded)"
        }
        return URL(string: string)
    }

    /// The relay's mirror page, for the in-app map view.
    var mirrorURL: URL? {
        guard let url = relayURL, var parts = URLComponents(url: url, resolvingAgainstBaseURL: false) else { return nil }
        parts.scheme = "http"
        parts.path = "/view.html"
        return parts.url
    }

    // MARK: relay callbacks

    private func relayOpened() {
        DispatchQueue.main.async { UIApplication.shared.isIdleTimerDisabled = true }   // UIKit: main thread only
        motion.start()
        let timer = DispatchSource.makeTimerSource(queue: sendQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(50))
        timer.setEventHandler { [weak self] in self?.flushMotion() }
        timer.resume()
        imuTimer = timer

        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = [.sceneDepth]
        lastSent = 0
        streamStart = Date()
        configLock.lock(); configReceived = false; imuOnly = false; configLock.unlock()
        DispatchQueue.main.async { self.mode = "waiting for the page" }
        streaming = true
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        DispatchQueue.main.async {
            self.running = true
            self.sentFrames = 0
            self.status = "Streaming"
        }
    }

    private func relayClosed(_ message: String) {
        guard relayURL != nil else { return }
        stop("Disconnected: \(message)")
    }

    private func relayControl(_ action: String, _ reason: String?) {
        guard action == "stop" else { return }
        let why: String
        switch reason {
        case "buffer-full": why = "The Mac's SLAM fell too far behind. Move slower or lower the camera resolution, then start again."
        case "gap": why = "A frame gap broke the capture chain. Start again."
        default: why = "Stopped from the Mac."
        }
        stop(why)
    }

    // MARK: sending

    private func flushMotion() {
        if let text = motion.drainMessage() { relay.send(text: text) }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // runs on delegateQueue
        guard streaming, !busy, let depth = frame.sceneDepth, relay.canAcceptCamera else { return }
        let (config, received, imuOnly) = settings
        guard !imuOnly else { return }
        // Until the page says what it wants, send nothing (an older page that never answers gets raw frames).
        if !received && Date().timeIntervalSince(streamStart) < configGrace { return }
        let useLandmarks = received && config.landmarks
        let interval = useLandmarks ? config.frameInterval : cameraInterval
        guard frame.timestamp - lastSent >= interval - 0.01 else { return }
        busy = true
        lastSent = frame.timestamp

        let colorBuffer = frame.capturedImage
        let depthMap = depth.depthMap
        let confidenceMap = depth.confidenceMap
        let intrinsics = frame.camera.intrinsics
        let resolution = frame.camera.imageResolution
        let transform = frame.camera.transform
        let timestamp = frame.timestamp
        let tracking = frame.camera.trackingState

        sendQueue.async { [weak self] in
            guard let self else { return }
            let message: Data?
            if useLandmarks {
                message = self.landmarkMessage(color: colorBuffer, depth: depthMap, confidence: confidenceMap,
                                               intrinsics: intrinsics, resolution: resolution,
                                               timestamp: timestamp, config: config)
            } else {
                message = self.cameraMessage(color: colorBuffer, depth: depthMap, confidence: confidenceMap,
                                             intrinsics: intrinsics, resolution: resolution,
                                             transform: transform, timestamp: timestamp)
            }
            if let message {
                // The IMU the frame depends on must reach the Mac first.
                self.flushMotion()
                self.relay.send(data: message)
                DispatchQueue.main.async {
                    self.sentFrames += 1
                    self.detail = Self.describe(tracking)
                }
            }
            self.delegateQueue.async { self.busy = false }
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        stop("ARKit error: \(error.localizedDescription)")
    }

    private static func describe(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal: return "ARKit tracking: normal"
        case .notAvailable: return "ARKit tracking: not available"
        case .limited(let reason):
            switch reason {
            case .excessiveMotion: return "ARKit tracking limited: moving too fast"
            case .insufficientFeatures: return "ARKit tracking limited: too few features"
            case .initializing: return "ARKit tracking: initializing"
            case .relocalizing: return "ARKit tracking: relocalizing"
            @unknown default: return "ARKit tracking limited"
            }
        }
    }

    /// Selects the landmarks on the phone and packs only those (IPL1); the raster and depth map stay here.
    private func landmarkMessage(color: CVPixelBuffer, depth: CVPixelBuffer, confidence: CVPixelBuffer?,
                                 intrinsics: simd_float3x3, resolution: CGSize,
                                 timestamp: TimeInterval, config: LandmarkConfig) -> Data? {
        let colorSize = (w: Double(resolution.width), h: Double(resolution.height))
        guard let rgb = FrameResampler.rgb(from: color, config: config),
              let z16 = FrameResampler.z16(depth: depth, confidence: confidence, colorSize: colorSize, config: config)
        else { return nil }
        let set = Landmarks.extract(rgb: rgb, depth: z16, config: config)
        return Landmarks.packet(set: set, timestamp: timestamp, colorSize: (Int(resolution.width), Int(resolution.height)),
                                intrinsics: (Double(intrinsics.columns.0.x), Double(intrinsics.columns.1.y),
                                             Double(intrinsics.columns.2.x), Double(intrinsics.columns.2.y)),
                                config: config)
    }

    /// magic 'IPH1' (uint32 LE) | header length (uint32 LE) | JSON header | JPEG | float32 LE depth | uint8 confidence
    private func cameraMessage(color: CVPixelBuffer, depth: CVPixelBuffer, confidence: CVPixelBuffer?,
                               intrinsics: simd_float3x3, resolution: CGSize,
                               transform: simd_float4x4, timestamp: TimeInterval) -> Data? {
        let image = CIImage(cvPixelBuffer: color)
        guard let space = CGColorSpace(name: CGColorSpace.sRGB),
              let jpeg = ciContext.jpegRepresentation(of: image, colorSpace: space,
                                                      options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): jpegQuality])
        else { return nil }

        let depthData = Self.bytes(of: depth, bytesPerPixel: 4)
        let confidenceData = confidence.map { Self.bytes(of: $0, bytesPerPixel: 1) } ?? Data()
        let dw = CVPixelBufferGetWidth(depth), dh = CVPixelBufferGetHeight(depth)
        guard depthData.count == dw * dh * 4 else { return nil }

        let m = transform.columns
        let pose: [Double] = [m.0.x, m.0.y, m.0.z, m.0.w, m.1.x, m.1.y, m.1.z, m.1.w,
                              m.2.x, m.2.y, m.2.z, m.2.w, m.3.x, m.3.y, m.3.z, m.3.w].map { Double($0) }
        let header: [String: Any] = [
            "type": "camera",
            "t": timestamp,
            "color": ["width": Int(resolution.width), "height": Int(resolution.height),
                      "fx": Double(intrinsics.columns.0.x), "fy": Double(intrinsics.columns.1.y),
                      "cx": Double(intrinsics.columns.2.x), "cy": Double(intrinsics.columns.2.y)],
            "depth": ["width": dw, "height": dh],
            "jpegBytes": jpeg.count,
            "depthBytes": depthData.count,
            "confidenceBytes": confidenceData.count == dw * dh ? confidenceData.count : 0,
            "pose": pose,
        ]
        // JSONSerialization raises an uncatchable Objective-C exception on NaN/inf.
        guard pose.allSatisfy({ $0.isFinite }), timestamp.isFinite else { return nil }
        guard let headerData = try? JSONSerialization.data(withJSONObject: header) else { return nil }

        var message = Data(capacity: 8 + headerData.count + jpeg.count + depthData.count + confidenceData.count)
        var magic = UInt32(0x31485049).littleEndian
        var length = UInt32(headerData.count).littleEndian
        withUnsafeBytes(of: &magic) { message.append(contentsOf: $0) }
        withUnsafeBytes(of: &length) { message.append(contentsOf: $0) }
        message.append(headerData)
        message.append(jpeg)
        message.append(depthData)
        if confidenceData.count == dw * dh { message.append(confidenceData) }
        return message
    }

    /// Tightly packed copy of a pixel buffer's first plane (the buffer's rows may be padded).
    private static func bytes(of buffer: CVPixelBuffer, bytesPerPixel: Int) -> Data {
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return Data() }
        let width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer)
        let stride = CVPixelBufferGetBytesPerRow(buffer), rowBytes = width * bytesPerPixel
        var out = Data(capacity: rowBytes * height)
        for row in 0..<height {
            out.append(base.advanced(by: row * stride).assumingMemoryBound(to: UInt8.self), count: rowBytes)
        }
        return out
    }

    private func setStatus(_ text: String) { DispatchQueue.main.async { self.status = text } }
}
