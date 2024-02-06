#include <verilated.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <csignal>
#include <unordered_map>
#include <unistd.h>
#include "Vmips_core.h"
#include "verilated_fst_c.h"
#include "Vmips_core__Dpi.h"
#include "memory_driver.h"
#include "memory.h"

Vmips_core *top; // Instantiation of module
MemoryDriver *memory_driver;
Memory *memory;

int memory_debug = 0;
int stream_dump = 0;
int stream_print = 0;
int stream_check = 1;
const char *benchmark = "nqueens";
// std::string hexfiles_dir = "/home/linux/ieng6/cs148sp22/public";
std::string hexfiles_dir = "..";

vluint64_t main_time = 0; // Current simulation time
// This is a 64-bit integer to reduce wrap over issues and
// allow modulus.  This is in units of the timeprecision
// used in Verilog (or from --timescale-override)

double sc_time_stamp()
{                     // Called by $time in Verilog
    return main_time; // converts to double, to match
                      // what SystemC does
}

volatile std::sig_atomic_t interrupt = 0;
vluint64_t stop_time = 0;

void signal_handler(int signal)
{
    interrupt = signal;
}

std::unordered_map<std::string, unsigned int> stats;

void stats_event(const char *e)
{
    std::string s(e);
    stats[s]++;
}

unsigned int instruction_count = 0;

void pc_event(const int pc)
{
    if (stream_print)
        std::cout << "-- EVENT pc=" << std::hex << pc << std::endl;
    if (stream_dump)
    {
        static std::ofstream f("pc.txt");
        if (stream_dump >= 2)
            f << std::dec << main_time << " ";
        f << std::hex << pc << std::endl;
    }
    if (stream_check)
    {
        std::string fname(hexfiles_dir + "/hexfiles/"+ std::string(benchmark) +".pc.txt");
        static std::ifstream f(fname);
        if (!f.is_open())
        {
            std::cerr << "Failed to open file: " << fname << std::endl;
            exit(-1);
        }

        unsigned int expected_pc;
        if (!(f >> std::hex >> expected_pc))
        {
            std::cout << "\n!! Ran out of expected pc."
                         "\n!! More instructions are executed than expected"
                         "\n!! Additional pc="
                      << std::hex << pc << std::endl;
            std::raise(SIGINT);
        }
        else if (expected_pc != pc)
        {
            std::cout << "\n!! [" << std::dec << main_time << "] expected_pc=" << std::hex << expected_pc
                      << " mismatches pc=" << pc << std::endl;
            std::raise(SIGINT);
        }
    }
    instruction_count++;
}

unsigned int write_back_count = 0;

void wb_event(const int addr, const int data)
{
    if (stream_print)
        std::cout << "-- EVENT wb addr=" << std::hex << addr
                  << " data=" << data << std::endl;
    if (stream_dump)
    {
        static std::ofstream f("wb.txt");
        if (stream_dump >= 2)
            f << std::dec << main_time << " ";
        f << std::hex << addr << " " << data << std::endl;
    }
    if (stream_check)
    {
        std::string fname(hexfiles_dir + "/hexfiles/"+ std::string(benchmark) +".wb.txt");
        static std::ifstream f(fname);
        if (!f.is_open())
        {
            std::cerr << "Failed to open file: " << fname << std::endl;
            exit(-1);
        }

        unsigned int expected_addr, expected_data;
        if (!(f >> std::hex >> expected_addr >> expected_data))
        {
            std::cout << "\n!! Ran out of expected write back."
                         "\n!! More write back are executed than expected"
                         "\n!! Additional write back addr="
                      << std::hex << addr << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
        else if (expected_addr != addr || expected_data != data)
        {
            std::cout << "\n!! [" << std::dec << main_time << "] expected write back mismatches"
                      << "\n!! [" << std::dec << main_time << "] expected addr=" << std::hex << expected_addr
                      << " data=" << expected_data
                      << "\n!! [" << std::dec << main_time << "] actual   addr=" << std::hex << addr
                      << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
    }

    write_back_count++;
}

unsigned int load_store_count = 0;
void ls_event(const int op, const int addr, const int data)
{
    if (stream_print)
        std::cout << "-- EVENT ls op=" << std::hex << op
                  << " addr=" << addr
                  << " data=" << data << std::endl;
    if (stream_dump)
    {
        static std::ofstream f("ls.txt");
        if (stream_dump >= 2)
            f << std::dec << main_time << " ";
        f << std::hex << op << " " << addr << " " << data << std::endl;
    }
    if (stream_check)
    {
        std::string fname(hexfiles_dir + "/hexfiles/"+ std::string(benchmark) +".ls.txt");
        static std::ifstream f(fname);
        if (!f.is_open())
        {
            std::cerr << "Failed to open file: " << fname << std::endl;
            exit(-1);
        }

        unsigned int expected_op, expected_addr, expected_data;
        if (!(f >> std::hex >> expected_op && f >> expected_addr && f >> expected_data))
        {
            std::cout << "\n!! Ran out of expected load store"
                         "\n!! More load store are executed than expected"
                         "\n!! Additional load store op="
                      << std::hex << op << " addr=" << addr << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
        else if (expected_op != op || expected_addr != addr || expected_data != data)
        {
            std::cout << "\n!! [" << std::dec << main_time << "] expected load store mismatches"
                      << "\n!! [" << std::dec << main_time << "] expected op=" << std::hex << expected_op
                      << " addr=" << expected_addr
                      << " data=" << expected_data
                      << "\n!! [" << std::dec << main_time << "] actual   op=" << std::hex << op
                      << " addr=" << addr
                      << " data=" << data << std::endl;
            std::raise(SIGINT);
        }
    }

    load_store_count++;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    int opt;
    int dump = 0;
    double memory_delay_factor = 1.0;
    while ((opt = getopt(argc, argv, "dmpstf:b:")) != -1)
    {
        switch (opt)
        {
        case 'd':
            // Dump verilog waves to simx.fst
            dump = 1;
            break;
        case 'm':
            // Print debug info for cpp memory model
            // Repeat to increase verbose level
            memory_debug++;
            break;
        case 'p':
            // Print stream events to stdout
            stream_print = 1;
            break;
        case 's':
            // Skip stream checks
            stream_check = 0;
            break;
        case 't':
            // Trace streams and save to files
            // Repeat to include time in the trace
            stream_dump++;
            break;
        case 'f':
            // Set memory delay factor
            {
                std::stringstream argument(optarg);
                argument >> memory_delay_factor;
            }
            break;
        case 'b':
            benchmark = optarg;
            break;
        default: /* '?' */
            std::cerr << "Usage: " << argv[0] << " [-dmpst] [-b benchmark] [+plusargs]" << std::endl;
            return -1;
        }
    }

    Verilated::commandArgs(argc, argv); // Remember args

    top = new Vmips_core; // Create instance
    std::string const hex_file_name (hexfiles_dir + "/hexfiles/" + std::string(benchmark) + ".hex");
    memory = new Memory(hex_file_name.c_str(), memory_delay_factor);
    memory_driver = new MemoryDriver(top, memory);

    VerilatedFstC *tfp;
    if (dump)
    {
        std::cout << "Dumping waveform to simx.fst\n";
        Verilated::traceEverOn(true);
        tfp = new VerilatedFstC;
        top->trace(tfp, 1024);
        tfp->open("simx.fst");
    }

    top->clk = 0;
    top->rst_n = 0;
    memory_driver->drive_reset();

    while (!top->done && !(interrupt && main_time >= stop_time))
    {
        top->clk = !top->clk; // Toggle clock
        if (top->clk)
            memory_driver->consume(main_time);
        if (main_time == 100)
            top->rst_n = 1; // Deassert reset
        top->eval();        // Evaluate model
        if (top->clk)
        {
            memory_driver->drive(main_time);
            memory->process(main_time);
        }
      //  if (main_time % 1000000 == 0)
        //    std::cout << "Time is now: " << main_time << std::endl;
        if (dump)
            tfp->dump(main_time);

        main_time += 5; // Time passes...

        if (interrupt && stop_time == 0)
        {
            stop_time = main_time + 1000;
            std::cerr << "\n!! Interrupt raised at time=" << main_time << std::endl
                      << "!! Running additional 100 cycles before terminating at stop_time=" << stop_time << std::endl;
        }
    }

    top->final(); // Done simulating
    delete memory_driver;
    delete memory;
    delete top;

    if (dump)
    {
        tfp->close();
    }

    int cycle_count = main_time / 10;
    std::cout << std::dec
              << "\n\nTotal time: " << main_time
              << "\nCycle count: " << cycle_count
              << "\nInstruction count: " << instruction_count
              << "\nCPI: " << (float)cycle_count / instruction_count << " IPC: " << (float)instruction_count / cycle_count << std::endl;

    std::cout << "\n== Stats ===============\n";

    for (const auto &e : stats)
        std::cout << e.first << ": " << e.second << std::endl;

    if (interrupt)
        std::cerr << "\n== ABORTED =============\nSimulation aborted at stop_time=" << main_time << std::endl;
}
