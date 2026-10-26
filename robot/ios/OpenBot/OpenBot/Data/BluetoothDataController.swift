//
// Created by Nitish Yadav on 29/08/22.
//

import Foundation
import CoreBluetooth
import CoreMotion

/// This class provides a BLE communication interface with the OpenBot vehicle
class bluetoothDataController: CMDeviceMotion, CBCentralManagerDelegate, CBPeripheralDelegate {
    static let shared: bluetoothDataController = bluetoothDataController()
    var centralManager: CBCentralManager?
    var tempCentralManager: CBCentralManager?
    var peri: CBPeripheral?
    var peripherals = Array<CBPeripheral>()
    var discoveredPeripheral: CBPeripheral!
    var vehicleReady: Bool = false
    var sonarData: String = ""
    var voltageDivider: String = ""
    var speedometer: String = ""
    var bumperData: String = ""
    var writeCharacteristics: CBCharacteristic?
    var robotInfo: String = ""
    /// The eight proximity octants from the `p` channel, fed to object avoidance.
    var proximityData: String = ""
    /// Object avoidance, ArduPilot Rover style. Every `c` drive command is
    /// routed through it - gamepad, mission, inference and Blockly alike - the
    /// same way ArduPilot applies avoidance inside the navigation stack so no
    /// flight mode escapes it. `enabled` is the quick-disable switch.
    let driveGate = DriveGate()

    required init?(coder: NSCoder) {
        super.init()
        fatalError("init(coder:) has not been implemented")
    }

    /// Initialization routine
    override init() {
        super.init()
        driveGate.apply(settings: AvoidanceSettings.load())
        centralManager = CBCentralManager(delegate: self, queue: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(startNotification), name: Notification.Name("updateLabel"), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(startNotification), name: Notification.Name("updateSerialMonitor"), object: nil)
    }

    /// callback function when ble state is changed [CBManagerState]
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if (central.state == .poweredOn) {
            let options: [String: Any] = [CBCentralManagerScanOptionAllowDuplicatesKey: true]
            centralManager?.scanForPeripherals(withServices: [CBUUID(string: Constants.openbotService)], options: options);
        } else {
            print("bluetooth is off ")
        }
    }

    /// callback function when ble searches for the available devices and list them.
    ///
    /// - Parameters:
    ///   - central: ble manager
    ///   - peripheral: discovered device
    ///   - advertisementData: device data
    ///   - RSSI: device unique id
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if peripheral.name != nil {
            if !peripherals.contains(peripheral) {
                peripherals.append(peripheral)
            }
        }
    }

    /// callback function when ble is connected to a device
    ///
    /// - Parameters:
    ///   - central: ble manager
    ///   - peripheral: connected device
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("connected to", peripheral)
        isBluetoothConnected = true;
        NotificationCenter.default.post(name: .bluetoothConnected, object: nil);
        peripheral.discoverServices(nil)

    }

    /// callback function when ble connection is failed with a device
    ///
    /// - Parameters:
    ///   - central: ble manager
    ///   - peripheral: device connection
    ///   - error: error that occurred
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("connection was failed")
    }

    /// callback function when device disconnects from a device
    ///
    /// - Parameters:
    ///   - central: ble manager
    ///   - peripheral: disconnected device
    ///   - error:  error if any
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("disconnected from :", peripheral)
        isBluetoothConnected = false
        NotificationCenter.default.post(name: .bluetoothDisconnected, object: nil)
    }

    /// callback function when ble start searching for the ble services
    ///
    /// - Parameters:
    ///   - peripheral: searched device
    ///   - error: error if any.
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            for service in services {
                peripheral.discoverCharacteristics([], for: service)
            }
        }
    }

    /// callback function for reading descriptors
    ///
    /// - Parameters:
    ///   - peripheral: searched peripherals
    ///   - characteristic:
    ///   - error: errors if any
    func peripheral(_ peripheral: CBPeripheral, didDiscoverDescriptorsFor characteristic: CBCharacteristic, error: Error?) {
        if let des = characteristic.descriptors {
            for d in des {
                peripheral.readValue(for: d)
            }
        }
    }

    /// callback for reading characteristics
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if (characteristic.properties == .write || characteristic.properties == .writeWithoutResponse) {
                    writeCharacteristics = characteristic;
                } else if (characteristic.properties == .notify || characteristic.properties == .read) {
                    subscribeToNotifications(peripheral: discoveredPeripheral, characteristic: characteristic)
                    readValue(characteristic: characteristic)
                    peripheral.discoverDescriptors(for: characteristic)
                }
            }
        }
    }

    /// callback function to subscribe for notifications
    func subscribeToNotifications(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        peripheral.setNotifyValue(true, for: characteristic)
    }

    /// callback function for updating notification state
    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if error != nil {
            return
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        print("Data sent to :", peripheral.name as Any);
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor descriptor: CBDescriptor, error: Error?) {
        _ = descriptor.value
    }

    /// callback function for reading values from the connected device
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        // Deal with errors (if any)
        if let error = error {
            print("Error discovering characteristics: %s", error.localizedDescription)
            return
        }

        guard let characteristicData = characteristic.value, let stringFromData = String(data: characteristicData, encoding: .utf8) else {
            return
        }
        let header = stringFromData.prefix(1)
        bluetoothData = stringFromData
        NotificationCenter.default.post(name: .updateSerialMonitor, object: nil)
        NotificationCenter.default.post(name: .updateLabel, object: nil)

        switch (header) {
        case "r":
            vehicleReady = true
            break
        case "f":
            robotInfo = stringFromData
            break
        case "v":
            voltageDivider = stringFromData
            break
        case "s":
            sonarData = stringFromData
            break
        case "w":
            speedometer = stringFromData
            break
        case "b":
            bumperData = stringFromData
            break
        case "p":
            proximityData = stringFromData
            handleProximity(stringFromData)
            break
        case "m":
            // RC mode switch: full-screen smile in MANUAL, cry in HOLD / LOST.
            RcModeFaceOverlay.shared.handle(message: stringFromData)
            break
        default:
            break
        }
    }

    /// Ingest one `p` proximity report and let the avoidance law re-evaluate.
    ///
    /// Every new report re-sends the stored request, so a cleared blockage
    /// releases the robot without the driver re-moving the stick - and a
    /// newly-blocked direction stops the car without waiting for the next
    /// stick move either.
    /// Change the avoidance knobs, persist them and re-send the held command.
    ///
    /// The re-send is what makes the quick-disable switch work in a hurry: a car
    /// being held at a barrier resumes the moment it goes off, and a car driving
    /// at one stops the moment it goes on.
    func updateObjectAvoidance(_ settings: AvoidanceSettings) {
        let value = settings.clamped()
        value.save()
        let decision = driveGate.apply(settings: value)
        let left = Int((decision.left * 255).rounded())
        let right = Int((decision.right * 255).rounded())
        writeWire("c\(left),\(right)\n")
        NotificationCenter.default.post(
            name: .updateSpeedLabel,
            object: String(left) + "," + String(right)
        )
    }

    private func handleProximity(_ message: String) {
        let body = message.dropFirst()
        guard let sectors = Proximity.parseSectorsCm(String(body)) else { return }
        guard driveGate.proximity.setSectorsCm(sectors) else { return }
        let before = driveGate.lastDecision
        let after = driveGate.onProximityUpdate()
        if before.blocked != after.blocked || before.backing != after.backing {
            let left = Int((after.left * 255).rounded())
            let right = Int((after.right * 255).rounded())
            writeWire("c\(left),\(right)\n")
            NotificationCenter.default.post(
                name: .updateSpeedLabel,
                object: String(left) + "," + String(right)
            )
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didModifyServices invalidatedServices: [CBService]) {
        print(invalidatedServices)
    }

    /// function to read the value from characteristic
    func readValue(characteristic: CBCharacteristic) {
        discoveredPeripheral?.readValue(for: characteristic)
    }

    /// function to send data to the connected device
    ///
    /// Drive commands (`c<left>,<right>`) are routed through the object-avoidance
    /// gate first; every other command goes out untouched.
    ///
    /// - Parameter payload: string that should be sent to the connected device
    func sendData(payload: String) {
        writeWire(driveGate.applyWire(payload))
    }

    func sendDataFromJs(payloadData: String) {
        writeWire(driveGate.applyWire(payloadData))
    }

    /// Write a payload straight to the robot, bypassing the avoidance gate.
    ///
    /// Used for commands that are already the law's output (the re-send after a
    /// proximity update) and for safety stops, which must never be vetoed or
    /// turned into a reverse.
    func writeWire(_ payload: String) {
        let dataToSend: Data? = payload.data(using: String.Encoding.utf8)
        if (dataToSend != nil && discoveredPeripheral != nil && discoveredPeripheral.canSendWriteWithoutResponse) {
            if let writeCharacteristics {
                discoveredPeripheral.writeValue(dataToSend!, for: writeCharacteristics, type: CBCharacteristicWriteType.withoutResponse)
            }
        }
    }

    func stopRobot() {
        driveGate.apply(left: 0, right: 0)
        let payLoad = "c0,0\n";
        let dataToSend: Data? = payLoad.data(using: String.Encoding.utf8);
        if discoveredPeripheral != nil {
            if let writeCharacteristics {
                discoveredPeripheral.writeValue(dataToSend!, for: writeCharacteristics, type: CBCharacteristicWriteType.withResponse);
            }
        }
    }

    /// function to disconnect the connected device
    func disconnect() {
        centralManager?.cancelPeripheralConnection(discoveredPeripheral)
        startScan()
        peri = nil
    }

    ///function to connect the selected device
    func connect() {
        discoveredPeripheral = peri
        discoveredPeripheral.delegate = self
        centralManager?.connect(discoveredPeripheral)
        centralManager?.stopScan()
    }

    /// function to start scanning the available device
    func startScan() {
        let options: [String: Any] = [CBCentralManagerScanOptionAllowDuplicatesKey: true]
        switch centralManager?.state {
        case .poweredOn:
            centralManager?.scanForPeripherals(withServices: [CBUUID(string: Constants.openbotService), CBUUID(string: Constants.openbotService_RX), CBUUID(string: Constants.openbotService_TX)], options: options);
        default:
            break
        }

    }

    @objc func startNotification() {
        //        tempCentralManager = CBCentralManager(delegate: self, queue: nil)
    }

    /**
     return integer value sonar
     - Returns:
     */
    func getSonar() -> Int {
        if sonarData != "" {
            let index = sonarData.index(after: sonarData.startIndex)
            let actualSonarValue = min(Int(String(sonarData[index...])) ?? 0, 300)
            return actualSonarValue
        }
        return 0;

    }

    // Wheel RPM from BLE speed sensor.
    private func parseWheelRpm() -> (left: Int, right: Int)? {
        guard speedometer != "" else { return nil }
        let index_1 = speedometer.index(after: speedometer.startIndex)
        guard let indexOfComma = speedometer.firstIndex(of: ",") else { return nil }
        let index_2 = speedometer.index(before: indexOfComma)
        let leftString = String(speedometer[index_1...index_2]).trimmingCharacters(in: .whitespacesAndNewlines)
        let rightString = String(speedometer[speedometer.index(after: indexOfComma)...]).trimmingCharacters(in: .whitespacesAndNewlines)
        guard let left = Float(leftString), let right = Float(rightString) else { return nil }
        return (Int(left), Int(right))
    }

    func getLeftWheelRpm() -> Int {
        parseWheelRpm()?.left ?? 0
    }

    func getRightWheelRpm() -> Int {
        parseWheelRpm()?.right ?? 0
    }

    func getFormattedRpmRobotInfo() -> String {
        guard let rpm = parseWheelRpm() else {
            return Strings.speedText
        }
        return Strings.speed + " (l,r) " + String(rpm.left) + "," + String(rpm.right) + " rpm"
    }

    func getFormattedRpmShort() -> String {
        guard let rpm = parseWheelRpm() else {
            return "---,--- rpm"
        }
        return String(rpm.left) + "," + String(rpm.right) + " rpm"
    }

    func getSpeed() -> Int {
        guard let rpm = parseWheelRpm() else { return 0 }
        return (rpm.left + rpm.right) / 2
    }

    /**
     return integer value voltage divider
     - Returns:
     */
    func getVoltage() -> Double {
        if voltageDivider != "" {
            let index = voltageDivider.index(after: voltageDivider.startIndex)
            return Double(String(voltageDivider[index...])) ?? 0
        }
        return 0.0;
    }

}


