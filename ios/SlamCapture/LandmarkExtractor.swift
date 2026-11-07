import CoreVideo
import Foundation

// On-phone landmark selection. The page (src/landmark-source.ts) is the reference for every rule
// here: FAST-9 scores on gray = (r+g+b)/3, corners without a usable LiDAR return are dropped, then
// the strongest corners are kept at least four pixels apart. Only the selected corners travel to the
// Mac, each with its 7x7 patch (sum of r+g+b per pixel) and its depth sample; the Mac never sees the image.

/// What the page asks for in its `config` message.
struct LandmarkConfig {
    var landmarks = true                // false: stream raw JPEG + depth as before
    var width = 640
    var height = 480
    var minConfidence = 1               // ARKit depth confidence below this reads as no return
    var nearMeters = 0.2
    var farMeters = 5.0
    var depthUnitsMeters = 0.001
    var capacity = 350
    var fastThreshold = 13.58
    var frameInterval: TimeInterval = 0.7

    init() {}

    init?(json: [String: Any]) {
        guard let width = json["width"] as? Int, let height = json["height"] as? Int, width >= 16, height >= 16 else { return nil }
        self.width = width
        self.height = height
        landmarks = (json["mode"] as? String) != "raw"
        minConfidence = json["minConfidence"] as? Int ?? 1
        nearMeters = (json["nearMeters"] as? NSNumber)?.doubleValue ?? 0.2
        farMeters = (json["farMeters"] as? NSNumber)?.doubleValue ?? 5.0
        depthUnitsMeters = (json["depthUnitsMeters"] as? NSNumber)?.doubleValue ?? 0.001
        capacity = json["landmarkCapacity"] as? Int ?? 350
        fastThreshold = (json["fastThreshold"] as? NSNumber)?.doubleValue ?? 13.58
        frameInterval = (json["frameIntervalSeconds"] as? NSNumber)?.doubleValue ?? 0.7
    }
}

struct LandmarkSet {
    var x: [UInt16] = []
    var y: [UInt16] = []
    var z: [UInt16] = []
    var score: [Float] = []
    var patch: [UInt16] = []            // 49 per landmark, row-major sums of r+g+b
    var count: Int { x.count }
}

enum Landmarks {
    static let patchWidth = 7
    static let patchRadius = 3
    static let patchSize = 49
    // The constants the native SLAM applies (RGBDFastInertialLocalizationStep).
    static let selectionNear = 0.28
    static let selectionFar = 10.0
    static let rankScale = 1e8
    static let suppressionRadius = 3
    static let border = 3

    /// Offsets (dx, dy) clockwise from the top of the radius-three circle.
    private static let circle: [(Int, Int)] = [
        (0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
        (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3),
    ]

    /// Round half to even, as the Modelica selector ranks scores.
    static func evenRound(_ value: Double) -> Double {
        let low = value.rounded(.down), fraction = value - low
        if fraction < 0.5 { return low }
        if fraction > 0.5 { return low + 1 }
        return low.truncatingRemainder(dividingBy: 2) == 0 ? low : low + 1
    }

    /// FAST-9 scores. Pixels that cannot reach `floorScore` (no two adjacent cardinal samples differ by
    /// at least that much, which every 9-sample arc requires) are left at zero, as the native program does.
    static func fastScores(rgb: [UInt8], width: Int, height: Int, floorScore: Double) -> [Double] {
        var gray = [Double](repeating: 0, count: width * height)
        for i in 0..<(width * height) {
            gray[i] = ((Double(rgb[3 * i]) + Double(rgb[3 * i + 1])) + Double(rgb[3 * i + 2])) / 3
        }
        var scores = [Double](repeating: 0, count: width * height)
        var offsets = [Int](repeating: 0, count: 16)
        for k in 0..<16 { offsets[k] = circle[k].1 * width + circle[k].0 }
        var d = [Double](repeating: 0, count: 16)
        for y in 3..<(height - 3) {
            for x in 3..<(width - 3) {
                let index = y * width + x
                let center = gray[index]
                // Cardinal pre-test: samples 0, 4, 8, 12 (top, right, bottom, left).
                if floorScore > 0 {
                    var possible = false
                    var previousBright = false, previousDark = false, firstBright = false, firstDark = false
                    for c in 0..<4 {
                        let diff = gray[index + offsets[4 * c]] - center
                        let bright = diff >= floorScore, dark = -diff >= floorScore
                        if c == 0 {
                            firstBright = bright; firstDark = dark
                        } else {
                            possible = possible || (previousBright && bright) || (previousDark && dark)
                        }
                        previousBright = bright; previousDark = dark
                    }
                    possible = possible || (previousBright && firstBright) || (previousDark && firstDark)
                    if !possible { continue }
                }
                for k in 0..<16 { d[k] = gray[index + offsets[k]] - center }
                var response = 0.0
                for start in 0..<16 {
                    var low = d[start], high = low
                    for offset in 1..<9 {
                        let v = d[(start + offset) & 15]
                        low = low < v ? low : v
                        high = high > v ? high : v
                    }
                    let dark = -high
                    let arc = low > dark ? low : dark
                    response = response > arc ? response : arc
                }
                scores[index] = response
            }
        }
        return scores
    }

    /// True where the depth sample is usable for a landmark (RGBDCalibratedPoint on identical grids).
    static func depthUsable(_ z16: UInt16, units: Double) -> Bool {
        let sample = Double(z16) * units
        return sample > selectionNear && sample < selectionFar - 0.05
    }

    /// The selector the native FAST program runs: depth-qualified scores, ranked by rounded score with
    /// raster-order ties, square suppression, early stop below the threshold.
    static func select(scores: [Double], depth: [UInt16], width: Int, height: Int,
                       threshold: Double, capacity: Int, depthUnits: Double) -> [Int] {
        var qualified = scores
        for i in 0..<qualified.count where qualified[i] > 0 && !depthUsable(depth[i], units: depthUnits) {
            qualified[i] = 0
        }
        let guardRank = (threshold * rankScale).rounded(.down) - 1
        var candidates: [(rank: Double, index: Int)] = []
        for y in border..<(height - border) {
            for x in border..<(width - border) {
                let index = y * width + x
                let rank = evenRound(qualified[index] * rankScale)
                if rank >= guardRank { candidates.append((rank, index)) }
            }
        }
        candidates.sort { $0.rank != $1.rank ? $0.rank > $1.rank : $0.index < $1.index }
        var occupied = [Bool](repeating: false, count: width * height)
        var picked: [Int] = []
        for candidate in candidates {
            if qualified[candidate.index] < threshold || picked.count >= capacity { break }
            if occupied[candidate.index] { continue }
            picked.append(candidate.index)
            let x = candidate.index % width, y = candidate.index / width
            for yy in max(border, y - suppressionRadius)...min(height - border - 1, y + suppressionRadius) {
                for xx in max(border, x - suppressionRadius)...min(width - border - 1, x + suppressionRadius) {
                    occupied[yy * width + xx] = true
                }
            }
        }
        return picked
    }

    /// Score, qualify by depth, select and describe, in that order.
    static func extract(rgb: [UInt8], depth: [UInt16], config: LandmarkConfig) -> LandmarkSet {
        let width = config.width, height = config.height
        let floorScore = max(0, (floor(config.fastThreshold * rankScale) - 2) / rankScale)
        let scores = fastScores(rgb: rgb, width: width, height: height, floorScore: floorScore)
        let picked = select(scores: scores, depth: depth, width: width, height: height,
                            threshold: config.fastThreshold, capacity: config.capacity,
                            depthUnits: config.depthUnitsMeters)
        var set = LandmarkSet()
        set.patch.reserveCapacity(picked.count * patchSize)
        for index in picked {
            let x = index % width, y = index / width
            set.x.append(UInt16(x))
            set.y.append(UInt16(y))
            set.z.append(depth[index])
            set.score.append(Float(scores[index]))
            for row in 0..<patchWidth {
                for column in 0..<patchWidth {
                    let p = ((y + row - patchRadius) * width + x + column - patchRadius) * 3
                    set.patch.append(UInt16(rgb[p]) + UInt16(rgb[p + 1]) + UInt16(rgb[p + 2]))
                }
            }
        }
        return set
    }

    /// magic 'IPL1' (uint32 LE) | header length (uint32 LE) | JSON header | x u16[n] | y u16[n] | z u16[n] | score f32[n] | patch u16[n*49]
    static func packet(set: LandmarkSet, timestamp: TimeInterval, colorSize: (Int, Int), intrinsics: (fx: Double, fy: Double, cx: Double, cy: Double),
                       config: LandmarkConfig) -> Data? {
        // JSONSerialization raises an uncatchable Objective-C exception on NaN/inf.
        guard timestamp.isFinite, [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy].allSatisfy({ $0.isFinite }) else { return nil }
        let header: [String: Any] = [
            "type": "landmarks",
            "t": timestamp,
            "count": set.count,
            "color": ["width": colorSize.0, "height": colorSize.1,
                      "fx": intrinsics.fx, "fy": intrinsics.fy, "cx": intrinsics.cx, "cy": intrinsics.cy],
            "target": ["width": config.width, "height": config.height],
        ]
        guard let headerData = try? JSONSerialization.data(withJSONObject: header) else { return nil }
        var message = Data(capacity: 8 + headerData.count + set.count * (2 + 2 + 2 + 4 + 2 * patchSize))
        func append<T: FixedWidthInteger>(_ value: T) {
            var little = value.littleEndian
            withUnsafeBytes(of: &little) { message.append(contentsOf: $0) }
        }
        append(UInt32(0x314C_5049))
        append(UInt32(headerData.count))
        message.append(headerData)
        for v in set.x { append(v) }
        for v in set.y { append(v) }
        for v in set.z { append(v) }
        for v in set.score { append(v.bitPattern) }
        for v in set.patch { append(v) }
        return message
    }
}

/// Crops the 4:3 (or any) ARKit frame to the target aspect and resamples it onto the target grid.
enum FrameResampler {
    /// Largest centered rectangle of the source with the destination's aspect ratio (cropFor in live-source.ts).
    static func crop(srcW: Double, srcH: Double, dstW: Double, dstH: Double) -> (x: Double, y: Double, w: Double, h: Double) {
        let srcAspect = srcW / srcH, dstAspect = dstW / dstH
        if srcAspect > dstAspect {
            let w = srcH * dstAspect
            return ((srcW - w) / 2, 0, w, srcH)
        }
        let h = srcW / dstAspect
        return (0, (srcH - h) / 2, srcW, h)
    }

    /// ARKit's capturedImage: 420YpCbCr8 bi-planar full range. Each target pixel averages the source samples
    /// it covers (box filter), then converts BT.601 full range to RGB.
    static func rgb(from buffer: CVPixelBuffer, config: LandmarkConfig) -> [UInt8]? {
        guard CVPixelBufferGetPlaneCount(buffer) == 2 else { return nil }
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        guard let yBase = CVPixelBufferGetBaseAddressOfPlane(buffer, 0),
              let cBase = CVPixelBufferGetBaseAddressOfPlane(buffer, 1) else { return nil }
        let srcW = CVPixelBufferGetWidthOfPlane(buffer, 0), srcH = CVPixelBufferGetHeightOfPlane(buffer, 0)
        let yStride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 0), cStride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 1)
        let yPlane = yBase.assumingMemoryBound(to: UInt8.self), cPlane = cBase.assumingMemoryBound(to: UInt8.self)
        let width = config.width, height = config.height
        let rect = crop(srcW: Double(srcW), srcH: Double(srcH), dstW: Double(width), dstH: Double(height))
        let stepX = rect.w / Double(width), stepY = rect.h / Double(height)
        let samplesX = max(1, Int(stepX.rounded(.up))), samplesY = max(1, Int(stepY.rounded(.up)))
        var out = [UInt8](repeating: 0, count: width * height * 3)
        for ty in 0..<height {
            for tx in 0..<width {
                var ySum = 0.0, cbSum = 0.0, crSum = 0.0
                for sy in 0..<samplesY {
                    let v = rect.y + (Double(ty) + (Double(sy) + 0.5) / Double(samplesY)) * stepY
                    let row = min(srcH - 1, max(0, Int(v)))
                    for sx in 0..<samplesX {
                        let u = rect.x + (Double(tx) + (Double(sx) + 0.5) / Double(samplesX)) * stepX
                        let column = min(srcW - 1, max(0, Int(u)))
                        ySum += Double(yPlane[row * yStride + column])
                        let chroma = (row / 2) * cStride + (column / 2) * 2
                        cbSum += Double(cPlane[chroma])
                        crSum += Double(cPlane[chroma + 1])
                    }
                }
                let n = Double(samplesX * samplesY)
                let yy = ySum / n, cb = cbSum / n - 128, cr = crSum / n - 128
                let o = (ty * width + tx) * 3
                out[o] = clamp(yy + 1.402 * cr)
                out[o + 1] = clamp(yy - 0.344136 * cb - 0.714136 * cr)
                out[o + 2] = clamp(yy + 1.772 * cb)
            }
        }
        return out
    }

    private static func clamp(_ value: Double) -> UInt8 { UInt8(min(255, max(0, value.rounded()))) }

    /// Nearest-neighbour resample of ARKit's float32 depth (meters) into Z16 on the target grid; zero where
    /// there is no trustworthy return (depthToZ16 in live-source.ts).
    static func z16(depth: CVPixelBuffer, confidence: CVPixelBuffer?, colorSize: (w: Double, h: Double), config: LandmarkConfig) -> [UInt16]? {
        CVPixelBufferLockBaseAddress(depth, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depth, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(depth) else { return nil }
        let dw = CVPixelBufferGetWidth(depth), dh = CVPixelBufferGetHeight(depth)
        let dStride = CVPixelBufferGetBytesPerRow(depth)
        var confidenceBase: UnsafeMutableRawPointer?
        var confidenceStride = 0
        if let confidence {
            CVPixelBufferLockBaseAddress(confidence, .readOnly)
            confidenceBase = CVPixelBufferGetBaseAddress(confidence)
            confidenceStride = CVPixelBufferGetBytesPerRow(confidence)
        }
        defer { if let confidence { CVPixelBufferUnlockBaseAddress(confidence, .readOnly) } }
        let width = config.width, height = config.height
        let rect = crop(srcW: colorSize.w, srcH: colorSize.h, dstW: Double(width), dstH: Double(height))
        var out = [UInt16](repeating: 0, count: width * height)
        for y in 0..<height {
            let v = rect.y + (Double(y) + 0.5) * rect.h / Double(height)
            let row = min(dh - 1, max(0, Int(v * Double(dh) / colorSize.h)))
            let rowPointer = base.advanced(by: row * dStride).assumingMemoryBound(to: Float32.self)
            for x in 0..<width {
                let u = rect.x + (Double(x) + 0.5) * rect.w / Double(width)
                let column = min(dw - 1, max(0, Int(u * Double(dw) / colorSize.w)))
                if let confidenceBase {
                    let level = confidenceBase.advanced(by: row * confidenceStride + column).assumingMemoryBound(to: UInt8.self).pointee
                    if Int(level) < config.minConfidence { continue }
                }
                let z = Double(rowPointer[column])
                if !z.isFinite || z < config.nearMeters || z > config.farMeters { continue }
                out[y * width + x] = UInt16(min(65535, (z / config.depthUnitsMeters).rounded()))
            }
        }
        return out
    }
}
