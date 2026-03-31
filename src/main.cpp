#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "Metrics.hpp"
#include "OBDClient.hpp"
#include "OBDServer.hpp"
#include "PIDHandler.hpp"
#include "Simulator.hpp"
#include "VehicleState.hpp"

#ifdef DASHBOARD_ENABLED
#include "Dashboard.hpp"
#endif

static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

static void printUsage() {
    std::cout <<
        "CANSimulator - OBD-II / CAN Bus Simulator & Diagnostic Tool\n\n"
        "Usage:\n"
        "  cansimulator --simulate                  Start ECU simulator + server\n"
        "  cansimulator --query   <PID>             Query a single PID once\n"
        "  cansimulator --dump                      Dump all supported PIDs\n"
        "  cansimulator --bench   <PID> <count>     latency / throughput benchmark\n"
#ifdef DASHBOARD_ENABLED
        "  cansimulator --dashboard                 Live SDL2 dashboard with controls\n"
#endif
        "\nSupported PIDs:\n"
        "  RPM  SPEED  COOLANT_TEMP  ENGINE_LOAD  THROTTLE\n"
        "  INTAKE_TEMP  MAF  FUEL_LEVEL  RUNTIME  DIST_DTC\n\n"
        "Quick start (two terminals [1 / 2]):\n"
        "[1]  $ ./cansimulator --simulate\n"
        "[2]  $ ./cansimulator --query RPM\n"
        "[2]  $ ./cansimulator --bench RPM 1000\n";
}

static int runSimulate() {
    Logger::info("CANSimulator starting on vcan0");

    VehicleState state;
    Metrics      metrics;
    Simulator    sim(state);
    OBDServer    server(state, metrics);

    std::thread simThread   ([&] { sim.run();    });
    std::thread serverThread([&] { server.run(); });

    Logger::info("Simulator and OBDServer are now running");

    // print metrics every 5 secs
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        auto count = metrics.requestCount();
        if (count > 0) {
            Logger::info("Requests: " + std::to_string(count) + 
                " | Avg: "     + std::to_string(metrics.avgLatencyUs() / 1000.0).substr(0, 6) + " ms" + 
                " | MaxLat: "  + std::to_string(metrics.maxLatencyUs() / 1000.0).substr(0, 6) + " ms" + 
                " | RPM: "     + std::to_string(static_cast<int>(state.rpm.load())) + 
                " | Speed: "   + std::to_string(static_cast<int>(state.speed.load())) + " km/h");
        }
    }

    Logger::info("Shutting down...");
    state.running = false;
    server.stop();

    simThread.join();
    serverThread.join();
    Logger::info("CANSimulator stopped. Total requests served: "  + std::to_string(metrics.requestCount()));

    return 0;
}

static int runQuery(const std::string& pidName) {
    std::optional<PID> pid = pidFromName(pidName);
    if (!pid) {
        std::cerr << "Unknown PID: '" << pidName << "'. Run cansimulator --help for list of supported PIDs.\n";
        return 1;
    }

    OBDClient client;
    std::optional<QueryResult> result = client.query(*pid);
    if (!result) {
        std::cerr << "No response. Make sure cansimulator --simulate is running on vcan0\n";
        return 1;
    }
    
    std::cout << std::left << std::setw(26) << result->name << 
        std::right << std::setw(10) << std::fixed << std::setprecision(2) << 
        result->value << "  " << result->unit << "  (" << result->latencyUs / 1000.0 << " ms)\n";
    
    return 0;
}

static int runDump() {
    static const std::vector<PID> allPIDs = {
        PID::RPM, PID::SPEED, PID::COOLANT_TEMP, PID::ENGINE_LOAD,
        PID::THROTTLE, PID::INTAKE_TEMP, PID::MAF, PID::FUEL_LEVEL,
        PID::RUNTIME, PID::DIST_DTC,
    };

    OBDClient client;
    std::cout << "\n=== CANSimulator Vehicle State Dump ===\n";

    for (auto pid : allPIDs) {
        auto result = client.query(pid);

        if (result) {
            std::cout << std::left << std::setw(26) << result->name << 
                std::right << std::setw(10) << std::fixed << 
                std::setprecision(2) << result->value << "  " << result->unit << "\n";
        } 
        
        else {
            std::cout << std::left << std::setw(26) << pidToName(pid) << " [no response]\n";
        }
    }
    std::cout << "\n";

    return 0;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    if (argc < 2) { printUsage(); return 1; }

    std::string mode = argv[1];

    if (mode == "--simulate") {
        return runSimulate();
    } 
    
    else if (mode == "--query" && argc >= 3) {
        return runQuery(argv[2]);
    } 
    
    else if (mode == "--dump") {
        return runDump();
    } 
    
    else if (mode == "--bench" && argc >= 4) {
        auto pid = pidFromName(argv[2]);
        if (!pid) {
            std::cerr << "Unknown PID: '" << argv[2] << "'\n"; return 1;
        }

        int count = std::stoi(argv[3]);
        OBDClient client;
        client.benchmark(*pid, count);

        return 0;
    } 
    
    else if (mode == "--help" || mode == "-h") {
        printUsage(); return 0;
    }

#ifdef DASHBOARD_ENABLED
    else if (mode == "--dashboard") {
        Logger::info("Starting dashboard mode on vcan0");

        VehicleState state;
        state.manualControl = true;
        Metrics   metrics;
        Simulator sim(state);
        OBDServer server(state, metrics);

        std::thread simThread   ([&] { sim.run();    });
        std::thread serverThread([&] { server.run(); });

        try {
            Dashboard dash(state, metrics);
            dash.run();
        } catch (const std::exception& ex) {
            Logger::error(std::string("Dashboard error: ") + ex.what());
            state.running = false;
            server.stop();

            simThread.join();
            serverThread.join();

            return 1;
        }

        state.running = false;
        server.stop();

        simThread.join();
        serverThread.join();

        return 0;
    }
#endif

    else {
        std::cerr << "Unknown command, check cansimulator --help for usage\n";
        return 1;
    }
}
