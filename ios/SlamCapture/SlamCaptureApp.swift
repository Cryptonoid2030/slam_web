import SwiftUI

@main
struct SlamCaptureApp: App {
    @StateObject private var engine = CaptureEngine()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(engine)
                .preferredColorScheme(.dark)
        }
    }
}
