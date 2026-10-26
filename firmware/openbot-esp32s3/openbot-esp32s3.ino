// OpenBot firmware for an ESP32-S3 (N16R8) Off-Road 4WD Ackermann robot car.
//
// The real code is in openbot-esp32s3.cpp. It lives in a .cpp rather than here
// because the Arduino build runs every .ino through ctags to guess function
// prototypes, and that pass mis-generates the return types of the helpers in
// this sketch. A .cpp is handed straight to the compiler, so the prototypes
// declared at the top of that file are the ones actually used.
//
// See openbot-esp32s3.cpp for the protocol, wiring and calibration.
