//
// Generated file, do not edit! Created by opp_msgc 4.2 from Packet.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "Packet_m.h"

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




Register_Class(Packet);

Packet::Packet(const char *name, int kind) : cPacket(name,kind)
{
    this->srcAddr_var = 0;
    this->destAddr_var = 0;
    this->hopCount_var = 0;
    this->travelTime_var = 0;
    transientNodes_arraysize = 0;
    this->transientNodes_var = 0;
}

Packet::Packet(const Packet& other) : cPacket(other)
{
    transientNodes_arraysize = 0;
    this->transientNodes_var = 0;
    copy(other);
}

Packet::~Packet()
{
    delete [] transientNodes_var;
}

Packet& Packet::operator=(const Packet& other)
{
    if (this==&other) return *this;
    cPacket::operator=(other);
    copy(other);
    return *this;
}

void Packet::copy(const Packet& other)
{
    this->srcAddr_var = other.srcAddr_var;
    this->destAddr_var = other.destAddr_var;
    this->hopCount_var = other.hopCount_var;
    this->travelTime_var = other.travelTime_var;
    delete [] this->transientNodes_var;
    this->transientNodes_var = (other.transientNodes_arraysize==0) ? NULL : new int[other.transientNodes_arraysize];
    transientNodes_arraysize = other.transientNodes_arraysize;
    for (unsigned int i=0; i<transientNodes_arraysize; i++)
        this->transientNodes_var[i] = other.transientNodes_var[i];
}

void Packet::parsimPack(cCommBuffer *b)
{
    cPacket::parsimPack(b);
    doPacking(b,this->srcAddr_var);
    doPacking(b,this->destAddr_var);
    doPacking(b,this->hopCount_var);
    doPacking(b,this->travelTime_var);
    b->pack(transientNodes_arraysize);
    doPacking(b,this->transientNodes_var,transientNodes_arraysize);
}

void Packet::parsimUnpack(cCommBuffer *b)
{
    cPacket::parsimUnpack(b);
    doUnpacking(b,this->srcAddr_var);
    doUnpacking(b,this->destAddr_var);
    doUnpacking(b,this->hopCount_var);
    doUnpacking(b,this->travelTime_var);
    delete [] this->transientNodes_var;
    b->unpack(transientNodes_arraysize);
    if (transientNodes_arraysize==0) {
        this->transientNodes_var = 0;
    } else {
        this->transientNodes_var = new int[transientNodes_arraysize];
        doUnpacking(b,this->transientNodes_var,transientNodes_arraysize);
    }
}

int Packet::getSrcAddr() const
{
    return srcAddr_var;
}

void Packet::setSrcAddr(int srcAddr)
{
    this->srcAddr_var = srcAddr;
}

int Packet::getDestAddr() const
{
    return destAddr_var;
}

void Packet::setDestAddr(int destAddr)
{
    this->destAddr_var = destAddr;
}

int Packet::getHopCount() const
{
    return hopCount_var;
}

void Packet::setHopCount(int hopCount)
{
    this->hopCount_var = hopCount;
}

simtime_t Packet::getTravelTime() const
{
    return travelTime_var;
}

void Packet::setTravelTime(simtime_t travelTime)
{
    this->travelTime_var = travelTime;
}

void Packet::setTransientNodesArraySize(unsigned int size)
{
    int *transientNodes_var2 = (size==0) ? NULL : new int[size];
    unsigned int sz = transientNodes_arraysize < size ? transientNodes_arraysize : size;
    for (unsigned int i=0; i<sz; i++)
        transientNodes_var2[i] = this->transientNodes_var[i];
    for (unsigned int i=sz; i<size; i++)
        transientNodes_var2[i] = 0;
    transientNodes_arraysize = size;
    delete [] this->transientNodes_var;
    this->transientNodes_var = transientNodes_var2;
}

unsigned int Packet::getTransientNodesArraySize() const
{
    return transientNodes_arraysize;
}

int Packet::getTransientNodes(unsigned int k) const
{
    if (k>=transientNodes_arraysize) throw cRuntimeError("Array of size %d indexed by %d", transientNodes_arraysize, k);
    return transientNodes_var[k];
}

void Packet::setTransientNodes(unsigned int k, int transientNodes)
{
    if (k>=transientNodes_arraysize) throw cRuntimeError("Array of size %d indexed by %d", transientNodes_arraysize, k);
    this->transientNodes_var[k] = transientNodes;
}

class PacketDescriptor : public cClassDescriptor
{
  public:
    PacketDescriptor();
    virtual ~PacketDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(PacketDescriptor);

PacketDescriptor::PacketDescriptor() : cClassDescriptor("Packet", "cPacket")
{
}

PacketDescriptor::~PacketDescriptor()
{
}

bool PacketDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<Packet *>(obj)!=NULL;
}

const char *PacketDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int PacketDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 5+basedesc->getFieldCount(object) : 5;
}

unsigned int PacketDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISARRAY | FD_ISEDITABLE,
    };
    return (field>=0 && field<5) ? fieldTypeFlags[field] : 0;
}

const char *PacketDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "srcAddr",
        "destAddr",
        "hopCount",
        "travelTime",
        "transientNodes",
    };
    return (field>=0 && field<5) ? fieldNames[field] : NULL;
}

int PacketDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='s' && strcmp(fieldName, "srcAddr")==0) return base+0;
    if (fieldName[0]=='d' && strcmp(fieldName, "destAddr")==0) return base+1;
    if (fieldName[0]=='h' && strcmp(fieldName, "hopCount")==0) return base+2;
    if (fieldName[0]=='t' && strcmp(fieldName, "travelTime")==0) return base+3;
    if (fieldName[0]=='t' && strcmp(fieldName, "transientNodes")==0) return base+4;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *PacketDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "simtime_t",
        "int",
    };
    return (field>=0 && field<5) ? fieldTypeStrings[field] : NULL;
}

const char *PacketDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int PacketDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    Packet *pp = (Packet *)object; (void)pp;
    switch (field) {
        case 4: return pp->getTransientNodesArraySize();
        default: return 0;
    }
}

std::string PacketDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    Packet *pp = (Packet *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getSrcAddr());
        case 1: return long2string(pp->getDestAddr());
        case 2: return long2string(pp->getHopCount());
        case 3: return double2string(pp->getTravelTime());
        case 4: return long2string(pp->getTransientNodes(i));
        default: return "";
    }
}

bool PacketDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    Packet *pp = (Packet *)object; (void)pp;
    switch (field) {
        case 0: pp->setSrcAddr(string2long(value)); return true;
        case 1: pp->setDestAddr(string2long(value)); return true;
        case 2: pp->setHopCount(string2long(value)); return true;
        case 3: pp->setTravelTime(string2double(value)); return true;
        case 4: pp->setTransientNodes(i,string2long(value)); return true;
        default: return false;
    }
}

const char *PacketDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldStructNames[] = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
    };
    return (field>=0 && field<5) ? fieldStructNames[field] : NULL;
}

void *PacketDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    Packet *pp = (Packet *)object; (void)pp;
    switch (field) {
        default: return NULL;
    }
}


