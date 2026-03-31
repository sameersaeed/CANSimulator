#include <gtest/gtest.h>
#include "PIDHandler.hpp"

// Validate encode / decode result matches input 

TEST(PIDRoundTrip, RPM_Midrange) { // expecting 0.25 rpm
    double v = 2500.0;
    auto enc = encodePid(PID::RPM, v);
    ASSERT_EQ(enc.numBytes, 2);
    auto res = decodeResponse(0x0C, enc.A, enc.B);
    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res->value, v, 0.25);
    EXPECT_EQ(res->unit, "rpm");
}

TEST(PIDRoundTrip, RPM_Idle) {
    double v = 800.0;
    auto enc = encodePid(PID::RPM, v);
    auto res = decodeResponse(0x0C, enc.A, enc.B);
    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res->value, v, 0.25);
}

TEST(PIDRoundTrip, RPM_Clamp_Max) { // values above max should be clamped (not wrapped)
    auto enc = encodePid(PID::RPM, 99999.0);
    auto res = decodeResponse(0x0C, enc.A, enc.B);
    ASSERT_TRUE(res.has_value());
    EXPECT_LE(res->value, 16384.0);
}

TEST(PIDRoundTrip, Speed) { // expecting 1 km/h
    for (double v : {0.0, 50.0, 120.0, 255.0}) {
        auto enc = encodePid(PID::SPEED, v);
        ASSERT_EQ(enc.numBytes, 1);
        auto res = decodeResponse(0x0D, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 0.5);
    }
}

TEST(PIDRoundTrip, CoolantTemp) { // expecting 1 °C, range: -40..+215 °C
    for (double v : {-40.0, 0.0, 90.0, 120.0}) {
        auto enc = encodePid(PID::COOLANT_TEMP, v);
        auto res = decodeResponse(0x05, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 0.5);
    }
}

TEST(PIDRoundTrip, EngineLoad) { // expecting ~0.39%, range: 0–100%
    for (double v : {0.0, 25.0, 50.0, 100.0}) {
        auto enc = encodePid(PID::ENGINE_LOAD, v);
        auto res = decodeResponse(0x04, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 1.0);
    }
}

TEST(PIDRoundTrip, Throttle) {
    for (double v : {5.0, 50.0, 100.0}) {
        auto enc = encodePid(PID::THROTTLE, v);
        auto res = decodeResponse(0x11, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 1.0);
    }
}

TEST(PIDRoundTrip, IntakeTemp) {
    for (double v : {-20.0, 25.0, 60.0}) {
        auto enc = encodePid(PID::INTAKE_TEMP, v);
        auto res = decodeResponse(0x0F, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 0.5);
    }
}

TEST(PIDRoundTrip, MAF) { // expecting 0.01 g/s
    for (double v : {1.5, 10.0, 25.0}) {
        auto enc = encodePid(PID::MAF, v);
        ASSERT_EQ(enc.numBytes, 2);
        auto res = decodeResponse(0x10, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 0.01);
    }
}

TEST(PIDRoundTrip, FuelLevel) {
    for (double v : {0.0, 50.0, 100.0}) {
        auto enc = encodePid(PID::FUEL_LEVEL, v);
        auto res = decodeResponse(0x2F, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_NEAR(res->value, v, 1.0);
    }
}

TEST(PIDRoundTrip, Runtime) { // exact seconds in int
    for (double v : {0.0, 300.0, 3600.0, 65535.0}) {
        auto enc = encodePid(PID::RUNTIME, v);
        ASSERT_EQ(enc.numBytes, 2);
        auto res = decodeResponse(0x1F, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_DOUBLE_EQ(res->value, v);
    }
}

TEST(PIDRoundTrip, DistDTC) {
    for (double v : {0.0, 1000.0, 65535.0}) {
        auto enc = encodePid(PID::DIST_DTC, v);
        auto res = decodeResponse(0x31, enc.A, enc.B);
        ASSERT_TRUE(res.has_value());
        EXPECT_DOUBLE_EQ(res->value, v);
    }
}

////////////////////////////////////////////////////////////////////////////////

// PID name resolution

TEST(PIDNameLookup, ValidNames) {
    EXPECT_EQ(pidFromName("RPM"),          PID::RPM);
    EXPECT_EQ(pidFromName("SPEED"),        PID::SPEED);
    EXPECT_EQ(pidFromName("COOLANT_TEMP"), PID::COOLANT_TEMP);
    EXPECT_EQ(pidFromName("ENGINE_LOAD"),  PID::ENGINE_LOAD);
    EXPECT_EQ(pidFromName("THROTTLE"),     PID::THROTTLE);
    EXPECT_EQ(pidFromName("MAF"),          PID::MAF);
    EXPECT_EQ(pidFromName("FUEL_LEVEL"),   PID::FUEL_LEVEL);
    EXPECT_EQ(pidFromName("RUNTIME"),      PID::RUNTIME);
    EXPECT_EQ(pidFromName("DIST_DTC"),     PID::DIST_DTC);
}

TEST(PIDNameLookup, InvalidName) {
    EXPECT_FALSE(pidFromName("BANANA").has_value());
    EXPECT_FALSE(pidFromName("").has_value());
    EXPECT_FALSE(pidFromName("rpm").has_value()); // case-sensitive
}

TEST(PIDNameLookup, ToName) {
    EXPECT_EQ(pidToName(PID::RPM),   "RPM");
    EXPECT_EQ(pidToName(PID::SPEED), "SPEED");
}

////////////////////////////////////////////////////////////////////////////////

// Edge cases

TEST(DecodeResponse, UnknownPID_ReturnsNullopt) {
    EXPECT_FALSE(decodeResponse(0xAB, 0, 0).has_value());
    EXPECT_FALSE(decodeResponse(0xFF, 0, 0).has_value());
}

TEST(DecodeResponse, ZeroBytes_ValidPID) { // RPM with A=0, B=0 = 0 rpm
    auto res = decodeResponse(0x0C, 0, 0);
    ASSERT_TRUE(res.has_value());
    EXPECT_DOUBLE_EQ(res->value, 0.0);
}

TEST(DecodeResponse, MaxBytes_ValidPID) { // RPM with A=255, B=255 = (65535)/4 = 16383.75 rpm
    auto res = decodeResponse(0x0C, 255, 255);
    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res->value, 16383.75, 0.01);
}
