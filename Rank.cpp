#include "Rank.h"
#include "MemoryController.h"

using namespace std;
using namespace DRAMSim;

Rank::Rank(ostream &dramsim_log_) :
	id(-1),
	dramsim_log(dramsim_log_),
	isPowerDown(false),
	refreshWaiting(false),
	readReturnCountdown(0),
	banks(NUM_BANKS, Bank(dramsim_log_)),
	bankStates(NUM_BANKS, BankState(dramsim_log_))

{

	memoryController = NULL;
	outgoingDataPacket = NULL;
	dataCyclesLeft = 0;
	currentClockCycle = 0;

#ifndef NO_STORAGE
#endif

}

// mutators
void Rank::setId(int id)
{
	this->id = id;
}

// attachMemoryController() must be called before any other Rank functions
// are called
void Rank::attachMemoryController(MemoryController *memoryController)
{
	this->memoryController = memoryController;
}
Rank::~Rank()
{
	for (size_t i=0; i<readReturnPacket.size(); i++)
	{
		delete readReturnPacket[i];
	}
	readReturnPacket.clear(); 
	delete outgoingDataPacket; 
}
void Rank::receiveFromBus(BusPacket *packet)
{

	switch (packet->busPacketType)
	{
	case READ_P:
		//make sure a read is allowed
//		if (bankStates[packet->bank].currentBankState != Idle ||currentClockCycle < bankStates[packet->bank].nextRead)
//		{
//			ERROR("== Error - Rank " << id << " received a READ_P when not allowed");
//			exit(-1);
//		}

		//update state table
//		bankStates[packet->bank].currentBankState = Idle;
//		bankStates[packet->bank].nextActivate = max(bankStates[packet->bank].nextActivate, currentClockCycle + MRAM_READ_TIME);

		//get the read data and put it in the storage which delays until the appropriate time (RL)
#ifndef NO_STORAGE
		banks[packet->bank].read(packet);
#else
		packet->busPacketType = DATA;
#endif

		readReturnPacket.push_back(packet);
		readReturnCountdown.push_back(MRAM_READ_TIME);
		break;

	case WRITE_P:
		//make sure a write is allowed
//		if (bankStates[packet->bank].currentBankState != Idle ||currentClockCycle < bankStates[packet->bank].nextWrite)
//		{
//			ERROR("== Error - Rank " << id << " received a WRITE_P when not allowed");
//			exit(0);
//		}

//		//update state table
//		bankStates[packet->bank].currentBankState = Idle;
//		bankStates[packet->bank].nextActivate = max(bankStates[packet->bank].nextActivate, currentClockCycle + MRAM_WRITE_TIME);

//taohi: in fact,case WRITE_P can be deleted here.
		incomingWriteBank = packet->bank;
		incomingWriteRow = packet->row;
		incomingWriteColumn = packet->column;
		delete(packet);
		break;
		
	case DATA:
		// TODO: replace this check with something that works?
		/*
		if(packet->bank != incomingWriteBank ||
			 packet->row != incomingWriteRow ||
			 packet->column != incomingWriteColumn)
			{
				cout << "== Error - Rank " << id << " received a DATA packet to the wrong place" << endl;
				packet->print();
				bankStates[packet->bank].print();
				exit(0);
			}
		*/
#ifndef NO_STORAGE
		banks[packet->bank].write(packet);
#else
		// end of the line for the write packet
#endif
		delete(packet);
		break;
	default:
		ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
		exit(0);
		break;
	}
}

int Rank::getId() const
{
	return this->id;
}

void Rank::update()
{

	// An outgoing packet is one that is currently sending on the bus
	// do the book keeping for the packet's time left on the bus
	if (outgoingDataPacket != NULL)
	{
		dataCyclesLeft--;
		if (dataCyclesLeft == 0)
		{
			//if the packet is done on the bus, call receiveFromBus and free up the bus
//			bankStates[outgoingDataPacket->bank].currentBankState = Idle;
			memoryController->receiveFromBus(outgoingDataPacket);
			outgoingDataPacket = NULL;
		}
	}

	// decrement the counter for all packets waiting to be sent back
	for (size_t i=0;i<readReturnCountdown.size();i++)
	{
		readReturnCountdown[i]--;
	}


	if (readReturnCountdown.size() > 0 && readReturnCountdown[0]==0)
	{
		// RL time has passed since the read was issued; this packet is
		// ready to go out on the bus

		outgoingDataPacket = readReturnPacket[0];
		dataCyclesLeft = BL/2;

		// remove the packet from the ranks
		readReturnPacket.erase(readReturnPacket.begin());
		readReturnCountdown.erase(readReturnCountdown.begin());

		if (DEBUG_BUS)
		{
			PRINTN(" -- R" << this->id << " Issuing On Data Bus : ");
			outgoingDataPacket->print();
			PRINT("");
		}

	}
}

//power down the rank
void Rank::powerDown()
{
	//perform checks
	for (size_t i=0;i<NUM_BANKS;i++)
	{
		if (bankStates[i].currentBankState != Idle)
		{
			ERROR("== Error - Trying to power down rank " << id << " while not all banks are idle");
			exit(0);
		}

		bankStates[i].nextPowerUp = currentClockCycle + tCKE;
		bankStates[i].currentBankState = PowerDown;
	}

	isPowerDown = true;
}

//power up the rank
void Rank::powerUp()
{
	if (!isPowerDown)
	{
		ERROR("== Error - Trying to power up rank " << id << " while it is not already powered down");
		exit(0);
	}

	isPowerDown = false;

	for (size_t i=0;i<NUM_BANKS;i++)
	{
		if (bankStates[i].nextPowerUp > currentClockCycle)
		{
			ERROR("== Error - Trying to power up rank " << id << " before we're allowed to");
			ERROR(bankStates[i].nextPowerUp << "    " << currentClockCycle);
			exit(0);
		}
		bankStates[i].nextActivate = currentClockCycle + tXP;
		bankStates[i].currentBankState = Idle;
	}
}
