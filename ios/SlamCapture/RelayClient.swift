import Foundation

/// WebSocket link to the relay's /ingest endpoint (`npm run live` on the Mac).
/// Camera messages are binary, IMU batches are text; see docs/guides/live-iphone.md.
final class RelayClient: NSObject, URLSessionWebSocketDelegate {
    struct Peers { var runner = 0; var views = 0 }

    var onOpen: () -> Void = {}
    var onClose: (String) -> Void = { _ in }
    var onControl: (_ action: String, _ reason: String?) -> Void = { _, _ in }
    var onPeers: (Peers) -> Void = { _ in }
    var onConfig: ([String: Any]) -> Void = { _ in }

    private var session: URLSession?
    private var task: URLSessionWebSocketTask?
    private let lock = NSLock()
    private var outstanding = 0

    /// Camera frames are shed (never queued) while earlier sends are still in flight.
    var canAcceptCamera: Bool {
        lock.lock(); defer { lock.unlock() }
        return task != nil && outstanding < 3
    }

    func connect(url: URL) {
        disconnect()
        let session = URLSession(configuration: .default, delegate: self, delegateQueue: nil)
        let task = session.webSocketTask(with: url)
        task.maximumMessageSize = 32 * 1024 * 1024
        self.session = session
        self.task = task
        task.resume()
        receiveLoop(task)
    }

    func disconnect() {
        task?.cancel(with: .goingAway, reason: nil)
        task = nil
        session?.invalidateAndCancel()
        session = nil
        lock.lock(); outstanding = 0; lock.unlock()
    }

    func send(text: String) { send(.string(text)) }
    func send(data: Data) { send(.data(data)) }

    private func send(_ message: URLSessionWebSocketTask.Message) {
        guard let task else { return }
        lock.lock(); outstanding += 1; lock.unlock()
        task.send(message) { [weak self] error in
            guard let self else { return }
            self.lock.lock(); self.outstanding = max(0, self.outstanding - 1); self.lock.unlock()
            if let error { self.onClose("Send failed: \(error.localizedDescription)") }
        }
    }

    private func receiveLoop(_ task: URLSessionWebSocketTask) {
        task.receive { [weak self] result in
            guard let self, task === self.task else { return }
            switch result {
            case .failure(let error):
                self.onClose(error.localizedDescription)
            case .success(let message):
                if case .string(let text) = message { self.handle(text) }
                self.receiveLoop(task)
            }
        }
    }

    private func handle(_ text: String) {
        guard let data = text.data(using: .utf8),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let type = json["type"] as? String else { return }
        switch type {
        case "control":
            onControl(json["action"] as? String ?? "", json["reason"] as? String)
        case "peers":
            onPeers(Peers(runner: json["runner"] as? Int ?? 0, views: json["views"] as? Int ?? 0))
        case "config":
            onConfig(json)
        default:
            break
        }
    }

    // MARK: URLSessionWebSocketDelegate
    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didOpenWithProtocol protocol: String?) {
        onOpen()
    }

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didCloseWith closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        onClose("Relay closed the connection (\(closeCode.rawValue))")
    }

    func urlSession(_ session: URLSession, task: URLSessionTask, didCompleteWithError error: Error?) {
        if let error { onClose(error.localizedDescription) }
    }
}
