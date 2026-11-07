import SwiftUI
import WebKit

struct ContentView: View {
    @EnvironmentObject var engine: CaptureEngine
    @AppStorage("relayAddress") private var address = ""
    @AppStorage("relayToken") private var token = ""
    @State private var showMap = false

    var body: some View {
        NavigationStack {
            Form {
                Section("Relay on your Mac") {
                    TextField("192.168.1.20:8787", text: $address)
                        .keyboardType(.URL)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .disabled(engine.running)
                    SecureField("Token (optional)", text: $token)
                        .textInputAutocapitalization(.never)
                        .disabled(engine.running)
                    Button(engine.running ? "Stop" : "Start streaming") {
                        if engine.running { engine.stop() } else { engine.start(address: address, token: token) }
                    }
                    .tint(engine.running ? .red : .green)
                }
                Section("Status") {
                    LabeledContent("State", value: engine.status)
                    if engine.running {
                        LabeledContent("Frames sent", value: "\(engine.sentFrames)")
                        LabeledContent("Sending", value: engine.mode)
                        LabeledContent("SLAM on the Mac", value: engine.runnerConnected ? "connected" : "not running")
                        LabeledContent("Mirror viewers", value: "\(engine.mirrorViews)")
                        if !engine.detail.isEmpty { Text(engine.detail).font(.footnote).foregroundStyle(.secondary) }
                    }
                    if !engine.lidarSupported {
                        Text("LiDAR scene depth is unavailable on this device.").foregroundStyle(.red)
                    }
                }
                Section {
                    Button("Show map") { showMap = true }.disabled(!engine.running)
                } footer: {
                    Text("Walk slowly and keep the phone steady. The SLAM runs on the Mac a few times slower than real time; the map catches up when you pause.")
                }
            }
            .navigationTitle("SLAM Capture")
            .sheet(isPresented: $showMap) {
                if let url = engine.mirrorURL { MapWebView(url: url).ignoresSafeArea() }
            }
        }
    }
}

/// The relay's Safari mirror page, embedded.
struct MapWebView: UIViewRepresentable {
    let url: URL
    func makeUIView(context: Context) -> WKWebView {
        let view = WKWebView(frame: .zero, configuration: WKWebViewConfiguration())
        view.isOpaque = false
        view.load(URLRequest(url: url))
        return view
    }
    func updateUIView(_ view: WKWebView, context: Context) {}
}
