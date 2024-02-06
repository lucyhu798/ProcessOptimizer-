#ifndef __INC__MEMORY_H__
#define __INC__MEMORY_H__

#include <iostream>
#include <cstdint>
#include <queue>

#define ADDR_WIDTH 26
#define DATA_WIDTH 32

#define AXI_ID_COUNT 16
#define AXI_MAX_ID (AXI_ID_COUNT - 1)

#define AXI_WRITE_ADDR_MAX_PENDING 8
#define AXI_WRITE_ADDR_MAX_PENDING_PER_ID 4
#define AXI_WRITE_DATA_MAX_BEATS 8
#define AXI_WRITE_DATA_MAX_PENDING (AXI_WRITE_ADDR_MAX_PENDING * AXI_WRITE_DATA_MAX_BEATS)
#define AXI_WRITE_DATA_MAX_PENDING_PER_ID (AXI_WRITE_ADDR_MAX_PENDING_PER_ID * AXI_WRITE_DATA_MAX_BEATS)

#define AXI_READ_ADDR_MAX_PENDING 8
#define AXI_READ_ADDR_MAX_PENDING_PER_ID 4
#define AXI_READ_DATA_MAX_BEATS 8
#define AXI_READ_DATA_MAX_PENDING (AXI_READ_ADDR_MAX_PENDING * AXI_READ_DATA_MAX_BEATS)
#define AXI_READ_DATA_MAX_PENDING_PER_ID (AXI_READ_ADDR_MAX_PENDING_PER_ID * AXI_READ_DATA_MAX_BEATS)

#define PUSH_OK 0
#define PUSH_FULL 1

struct AxiWriteAddress
{
    uint8_t awid, awlen;
    uint32_t awaddr;
    uint64_t time_start;

    bool committed;

    friend std::ostream &operator<<(std::ostream &os, const AxiWriteAddress &pkt)
    {
        return os << "AxiWriteAddress awid=" << std::dec << unsigned(pkt.awid) << " awlen=" << unsigned(pkt.awlen)
                  << " awaddr=" << std::hex << std::showbase << pkt.awaddr
                  << " time_start=" << std::dec << std::noshowbase << pkt.time_start << std::endl;
    }
};

struct AxiWriteData
{
    uint8_t wid, wlast;
    uint32_t wdata;
    uint64_t time_start;

    friend std::ostream &operator<<(std::ostream &os, const AxiWriteData &pkt)
    {
        return os << "AxiWriteData wid=" << std::dec << unsigned(pkt.wid) << " wlast=" << unsigned(pkt.wlast)
                  << " wdata=" << std::hex << std::showbase << pkt.wdata
                  << " time_start=" << std::dec << std::noshowbase << pkt.time_start << std::endl;
    }
};

struct AxiWriteResponse
{
    uint8_t bid;

    friend std::ostream &operator<<(std::ostream &os, const AxiWriteResponse &pkt)
    {
        return os << "AxiWriteResponse bid=" << std::dec << unsigned(pkt.bid) << std::endl;
    }
};

struct AxiReadAddress
{
    uint8_t arid, arlen;
    uint32_t araddr;
    uint64_t time_start;

    bool committed;

    friend std::ostream &operator<<(std::ostream &os, const AxiReadAddress &pkt)
    {
        return os << "AxiReadAddress arid=" << std::dec << unsigned(pkt.arid) << " arlen=" << unsigned(pkt.arlen)
                  << " araddr=" << std::hex << std::showbase << pkt.araddr
                  << " time_start=" << std::dec << std::noshowbase << pkt.time_start << std::endl;
    }
};

struct AxiReadData
{
    uint8_t rid, rlast;
    uint32_t rdata;

    friend std::ostream &operator<<(std::ostream &os, const AxiReadData &pkt)
    {
        return os << "AxiReadData rid=" << std::dec << unsigned(pkt.rid) << " rlast=" << unsigned(pkt.rlast)
                  << " rdata=" << std::hex << std::showbase << pkt.rdata << std::dec << std::noshowbase << std::endl;
    }
};

class Memory
{
public:
    Memory(const char *const hex_file, double delay_factor = 1.0);

    void process(uint64_t time);

    bool full_write_address() const;
    bool full_write_data() const;
    bool full_read_address() const;
    void push_write_address(const AxiWriteAddress &pkt);
    void push_write_data(const AxiWriteData &pkt);
    void push_read_address(const AxiReadAddress &pkt);

    const AxiWriteResponse *const peek_write_response() const;
    const AxiReadData *const peek_read_data() const;
    void pop_write_response();
    void pop_read_data();

    // Ingress pipe stage
    AxiWriteAddress *write_address_pipe;
    AxiWriteData *write_data_pipe;
    AxiReadAddress *read_address_pipe;

    // Ingress queues
    std::queue<AxiWriteAddress> write_address[AXI_ID_COUNT];
    std::queue<AxiWriteData> write_data[AXI_ID_COUNT];
    std::queue<AxiReadAddress> read_address[AXI_ID_COUNT];

    // Egress queues
    std::queue<AxiWriteResponse> write_response;
    std::queue<AxiReadData> read_data;

private:
    uint32_t m[1 << (ADDR_WIDTH - 2)];
    double delay_factor;

    void process_pipe();
    void process_read(uint64_t time);
    void process_write(uint64_t time);
    void commit_read(AxiReadAddress &pkt, uint64_t time);
    void commit_write(AxiWriteAddress &pkt, uint64_t time);
};

#endif
