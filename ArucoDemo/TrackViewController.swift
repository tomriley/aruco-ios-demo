//
//  TrackViewController.swift
//  ArucoDemo
//
//  Created by Carlo Rapisarda on 20/06/2018.
//  Copyright © 2018 Carlo Rapisarda. All rights reserved.
//

import UIKit
import AVFoundation
import SwiftSocket

class TrackViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate {

    @IBOutlet private weak var camView: UIImageView!
    @IBOutlet private weak var rotatedBar: RotatedBar!

    private var trackerSetup = false
    private var captureSession: AVCaptureSession?
    private var device: AVCaptureDevice?
    private var arucoTracker: ArucoTracker?
    
    let host = "192.168.1.76"
    let port = 8085
    var client: UDPClient?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        client = UDPClient(address: host, port: Int32(port))
        setupRotatedBar()
    }

    func setupRotatedBar() {
        let thisTextDown = UILabel(frame: .zero)
        thisTextDown.translatesAutoresizingMaskIntoConstraints = false
        thisTextDown.text = "This text down"
        thisTextDown.sizeToFit()
        rotatedBar.setup(with: [thisTextDown], insets: (dx: 0, dy: 10))
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        setupTrackerIfNeeded()
        captureSession?.startRunning()
    }

    override func viewDidDisappear(_ animated: Bool) {
        super.viewDidDisappear(animated)
        captureSession?.stopRunning()
    }

    func setupTrackerIfNeeded() {

        if trackerSetup {return}

        let docsDir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].path
        let calibPath = "\(docsDir)/camera_parameters.yml"

        if !FileManager.default.fileExists(atPath: calibPath) {
            warnUser()
        } else {
            arucoTracker = ArucoTracker(calibrationFile: calibPath, delegate: self)
            setupSession()
            trackerSetup = true
        }
    }

    func setupSession() {

        guard let tracker = arucoTracker
            else { fatalError("prepareSession() must be called after initializing the tracker") }

        device = AVCaptureDevice.default(for: .video)
        guard device != nil, let videoInput = try? AVCaptureDeviceInput(device: device!)
            else { fatalError("Device unavailable") }

        let videoOutput = AVCaptureVideoDataOutput()

        captureSession = AVCaptureSession()
        captureSession?.sessionPreset = .iFrame960x540

        guard captureSession!.canAddInput(videoInput), captureSession!.canAddOutput(videoOutput)
            else { fatalError("Cannot add video I/O to capture session") }

        captureSession?.addInput(videoInput)
        captureSession?.addOutput(videoOutput)

        tracker.previewRotation = .cw90
        tracker.prepare(for: videoOutput, orientation: .landscapeRight)
    }

    func showCalibrator(_ sender: Any) {
        let vcIndex = tabBarController?.viewControllers?.index { $0 is CalibratorViewController }
        tabBarController?.selectedIndex = Int(vcIndex!)
    }

    func warnUser() {
        let alert = UIAlertController(
            title: "Warning",
            message: "Camera calibration file (camera_parameters.yml) not found in Documents folder; this is necessary for accurate pose detection.",
            preferredStyle: .alert
        )
        alert.addAction(UIAlertAction(title: "Calibrate", style: .default, handler: showCalibrator))
        alert.addAction(UIAlertAction(title: "Ignore", style: .cancel, handler: nil))
        present(alert, animated: true, completion: nil)
    }
}

extension TrackViewController: ArucoTrackerDelegate {

    func arucoTracker(_ tracker: ArucoTracker, didDetect markers: [ArucoMarker], preview: UIImage?) {
        DispatchQueue.main.async { [unowned self] in
            self.camView.image = preview
            if let marker = markers.first {
                let str = String(format: "%f, %f, %f, %f, %f, %f, %f, %f, %f",
                                 marker.eyeX,
                                 marker.eyeY,
                                 marker.eyeZ,
                                 marker.dirX,
                                 marker.dirY,
                                 marker.dirZ,
                                 marker.upX,
                                 marker.upY,
                                 marker.upZ)
                self.client?.send(string: str)
            }
        }
    }
}
