//
// Generated file, do not edit! Created by opp_msgc 4.2 from Packet.msg.
//

#ifndef _PACKET_M_H_
#define _PACKET_M_H_

#include <omnetpp.h>

// opp_msgc version check
#define MSGC_VERSION 0x0402
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of opp_msgc: 'make clean' should help.
#endif



/**
 * Class generated from <tt>Packet.msg</tt> by opp_msgc.
 * <pre>
 * packet Packet
 * {
 *     int srcAddr;
 *     int destAddr; 
 *     int hopCount; 
 *     simtime_t travelTime;
 *     int transientNodes[]; 
 * }
 * </pre>
 */
class Packet : public ::cPacket
{
  protected:
    int srcAddr_var;
    int destAddr_var;
    int hopCount_var;
    simtime_t travelTime_var;
    int *transientNodes_var; // array ptr
    unsigned int transientNodes_arraysize;

  private:
    void copy(const Packet& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const Packet&);

  public:
    Packet(const char *name=NULL, int kind=0);
    Packet(const Packet& other);
    virtual ~Packet();
    Packet& operator=(const Packet& other);
    virtual Packet *dup() const {return new Packet(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual int getSrcAddr() const;
    virtual void setSrcAddr(int srcAddr);
    virtual int getDestAddr() const;
    virtual void setDestAddr(int destAddr);
    virtual int getHopCount() const;
    virtual void setHopCount(int hopCount);
    virtual simtime_t getTravelTime() const;
    virtual void setTravelTime(simtime_t travelTime);
    virtual void setTransientNodesArraySize(unsigned int size);
    virtual unsigned int getTransientNodesArraySize() const;
    virtual int getTransientNodes(unsigned int k) const;
    virtual void setTransientNodes(unsigned int k, int transientNodes);
};

inline void doPacking(cCommBuffer *b, Packet& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, Packet& obj) {obj.parsimUnpack(b);}


#endif // _PACKET_M_H_
