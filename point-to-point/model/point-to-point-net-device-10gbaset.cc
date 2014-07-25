/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007, 2008 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA	02111-1307	USA
 */

/* ***************************************************************************************************
 *																		      INCLUDES
 * ***************************************************************************************************/

#include "ns3/log.h"
#include "ns3/queue.h"
#include "ns3/simulator.h"
#include "ns3/mac48-address.h"
#include "ns3/llc-snap-header.h"
#include "ns3/error-model.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/mpi-interface.h"
#include "point-to-point-net-device.h"
#include "point-to-point-channel.h"
#include "ns3/ethernet-header.h"
#include "ethernet-trailer.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/command-line.h"
#include <iostream>

NS_LOG_COMPONENT_DEFINE ("PointToPointNetDevice");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PointToPointNetDevice);

/* ***************************************************************************************************
 *																		 LOCAL PROTOTYPES
 * ***************************************************************************************************/

static void m_TxSleepClockTrigger (PointToPointNetDevice *device); // Timer trigger : Sleep/Check

static void m_TxWakeUpTrigger (PointToPointNetDevice *device); // Timer trigger : Tx up
static void m_RxWakeUpTrigger (PointToPointNetDevice *device); // Timer trigger : Rx up

static void m_TxPowerDownTrigger (PointToPointNetDevice *device, Time startTime); // Timer trigger : Tx down
static void m_RxPowerDownTrigger (PointToPointNetDevice *device, Time startTime); // Timer trigger : Rx down

static void m_SendPCSMessageTrigger (PointToPointNetDevice *device, PointToPointNetDevice::PCSMessage msg); // Timer trigger : PCS send

/* ***************************************************************************************************
 *																		 UP & DOWN HIGH LEVEL
 * ***************************************************************************************************/

void PointToPointNetDevice::setTxLock(bool b)
{
  NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [ ] " << Simulator::Now ().GetSeconds() << "\t" << " Setting TxLock=" << std::boolalpha << b);
  TxLock = b;
}

void PointToPointNetDevice::setRxLock(bool b)
{
  NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [ ] " << Simulator::Now ().GetSeconds() << "\t" << " Setting RxLock=" << std::boolalpha << b);
  RxLock = b;
}

void
PointToPointNetDevice::NIC_UP ()
{
	if (m_eee == false)
    {
      // Cancel RX__UP if EEE disabled
      return;
    }
    
	NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [ ] " 
						<< "\t\t" << " NIC UP");
  
	// Change state UP
	ChangeState (RX_UP);
	ChangeState (TX_UP);
	// Schedule UP -> READY
	Simulator::Schedule (Seconds (m_t_up), &m_RxWakeUpTrigger, this);
	Simulator::Schedule (Seconds (m_t_up), &m_TxWakeUpTrigger, this);
}

void
PointToPointNetDevice::NIC_DOWN ()
{
	if (m_eee == false)
    {
      // Cancel RX__UP if EEE disabled
      return;
    }
  
	NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [ ] " 
								<<  "\t\t" << " NIC DOWN");
  
	// Change state DOWN
	ChangeState (TX_DOWN);
	ChangeState (RX_DOWN);
	// Schedule DOWN -> SLEEP
	Simulator::Schedule (Seconds (m_t_down), &m_TxPowerDownTrigger, this, Simulator::Now());
	Simulator::Schedule (Seconds (m_t_down), &m_RxPowerDownTrigger, this, Simulator::Now());
}


/* ***************************************************************************************************
 *																		 UP & DOWN LOW LEVEL
 * ***************************************************************************************************/

void
PointToPointNetDevice::m_TxWakeUp ()
{
	NS_ASSERT_MSG(m_txMachineState == TX_UP, " Expected TX_UP to wake up");
	
	if (m_txMachineState == TX_UP)
		{
			ChangeState (TX_READY);
			if (m_queue->GetNPackets() > 0)
			{
				initTransmissionIfPossible();
			}
		}
}

void
PointToPointNetDevice::m_RxWakeUp ()
{
	NS_ASSERT_MSG(m_rxMachineState == RX_UP, " Expected RX_UP to wake up");
	
	if (m_rxMachineState == RX_UP)
		{
			ChangeState (RX_READY);
		}
}

void
PointToPointNetDevice::m_TxPowerDown ()
{
	if (m_txMachineState == TX_DOWN)
		{
			ChangeState (TX_SLEEP);
		}
	else
		{
			// Queue threshold event occured in DOWN time: DOWN >> READY or BUSY
			return;
		}
}

void
PointToPointNetDevice::m_RxPowerDown ()
{
	if (m_rxMachineState == RX_DOWN)
		{
			ChangeState (RX_SLEEP);
		}
	else
		{
			// Queue threshold event occured in DOWN time: DOWN >> READY or BUSY
			return;
		}
}

/* ***************************************************************************************************
 *																			TX SLEEP CLOCK (LOW FREQUENCY)
 * ***************************************************************************************************/

void
PointToPointNetDevice::m_TxSleepClock ()
{
	//~ NS_ASSERT ((m_txMachineState == TX_SLEEP) || (m_txMachineState == TX_DOWN));
	
	if (isCounting == true)
	{
			m_ticks_count--;
			
			UintegerValue m_queue_capacity;
			m_queue->GetAttribute ("MaxPackets", m_queue_capacity);
			unsigned int queue_threshold = (unsigned int) ((double) ((double) m_queue_factor / (double) 100) * (double) m_queue_capacity.Get());
			
			if (m_ticks_count == 0)
				{
					// Timeout event
					event_timeout++;
					NS_LOG_DEBUG (m_address << "(" << TxLock << ", " << RxLock << ") [!] " 
									<< "\t" << " (timeout - " << m_queue->GetNPackets () << "/" << queue_threshold << " packets - " 
									<< (m_ticks_max - m_ticks_count) << "/" << m_ticks_max << " ticks)");
									
					// Disable 1st timer out
					isCounting = false;
					
					// Start transmission
					NIC_UP ();
					SendPCSMessage(PCS_UP);
				}

			else
				{
					// Continue counting
					Simulator::Schedule (Seconds (beta * m_t_up), &m_TxSleepClockTrigger, this);
				}

		}
	else
		{
			// Nothing in queue yet
			Simulator::Schedule (Seconds (beta * m_t_up), &m_TxSleepClockTrigger, this);
		}

}

/* ***************************************************************************************************
 *																				TIMER TRIGGERS
 * ***************************************************************************************************/

static void
m_TxWakeUpTrigger (PointToPointNetDevice *device)
{
	NS_LOG_FUNCTION (device);
	device->m_TxWakeUp();
}

static void
m_RxWakeUpTrigger (PointToPointNetDevice *device)
{
	NS_LOG_FUNCTION (device);
	device->m_RxWakeUp();
}

static void
m_TxPowerDownTrigger (PointToPointNetDevice *device, Time startTime)
{
	if (device->getLastTxUpdateTime() == startTime)
	{
		NS_LOG_FUNCTION (device);
		device->m_TxPowerDown();
	}
}

static void
m_RxPowerDownTrigger (PointToPointNetDevice *device, Time startTime)
{
	if (device->getLastRxUpdateTime() == startTime)
	{
		NS_LOG_FUNCTION (device);
		device->m_RxPowerDown();
	}
}

static void
m_TxSleepClockTrigger (PointToPointNetDevice *device)
{
	device->m_TxSleepClock();
}

/* ***************************************************************************************************
 *																	    	 EMULATION PCS
 * ***************************************************************************************************/

Ptr<PointToPointNetDevice>
PointToPointNetDevice::GetRemoteNetDevice (void) const
{
	NS_ASSERT (m_channel->GetNDevices () == 2);

	if (m_channel->GetPointToPointDevice (0) == this)
		{
			return m_channel->GetPointToPointDevice (1);
		}
	else
		{
			return m_channel->GetPointToPointDevice (0);
		}

}

bool
PointToPointNetDevice::IsLocalReceiveOn()
{
	return ((m_rxMachineState == RX_UP) || (m_rxMachineState == RX_READY) || (m_rxMachineState == RX_BUSY));
}

void
PointToPointNetDevice::SendPCSMessage (PCSMessage msg)
{
	// Warning: needs to acces channel propagation delay (set to public in PointToPointChannel)
	Simulator::Schedule (m_channel->GetDelay(), &m_SendPCSMessageTrigger, this, msg);
}

static void
m_SendPCSMessageTrigger (PointToPointNetDevice *device, PointToPointNetDevice::PCSMessage msg)
{
	device->SendPCSMessageNow(msg);
}

void
PointToPointNetDevice::SendPCSMessageNow (PCSMessage msg)
{
	GetRemoteNetDevice()->ReceivePCSMessage(msg);
}

void
PointToPointNetDevice::ReceivePCSMessage (PCSMessage msg)
{
	// PCS message received
	NS_LOG_FUNCTION_NOARGS ();

	switch (msg)
		{
				case PointToPointNetDevice::PCS_UP:
					{
						NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [P] " 
									<< "\t" << " PCS Message received : PCS UP");
									
						// Two devices can wake up at the same time => both is UP and receive PCS UP at same time
						// We don't have to wake up in this case.
						if (m_txMachineState == TX_SLEEP)
							{
								NIC_UP ();
							}
						if (m_txMachineState == TX_DOWN)
							{
								ChangeState (RX_READY);
								ChangeState (TX_READY);
							}
					}
					break;

				case PointToPointNetDevice::PCS_DOWN:
					{
						NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [P] " 
										<< "\t" << " PCS Message received : PCS DOWN");
										
						// The remote device ceased transmission.
						if (RxLock)
							{
								setRxLock(false);
							}
						
						// Device is ready to sleep?
						if (!TxLock)
							{
								NIC_DOWN ();
							}
					}
					break;

				default:
					{
						NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [P] " 
									<< "\t" << " PCS Message received : ???");
					}

		}

}

/* ***************************************************************************************************
 *																		 CONSTRUCTOR
 * ***************************************************************************************************/

PointToPointNetDevice::PointToPointNetDevice ()
{
	NS_LOG_FUNCTION (this);

	// Machine state (TX & RX) and update times
	m_txMachineState = TX_SLEEP;
	m_rxMachineState = RX_SLEEP;
	m_lastTxUpdateTime = Seconds (0.0);
	m_lastRxUpdateTime = Seconds (0.0);

	// Link variables
	m_channel = 0;
	m_eee = true; // EEE enabled by default
	m_linkUp = false;

	// Current packet handled
	m_currentPkt = 0;

	// LPI variables
	m_ticks_count = -1;
	beta = ((m_txPower - m_sleepPower) / (m_idlePower - m_sleepPower)) - 1;
	//~ Simulator::ScheduleNow (&m_TxSleepClockTrigger, this);

	// Stats
	m_sentPackets = 0;
	m_droppedPackets = 0;
	m_receivedPackets = 0;
	m_receivingError = 0;
	event_queue = 0;
	event_timeout = 0;

	TxLock = false;
	RxLock = false;

}

void
PointToPointNetDevice::Reset ()
{

	// Reset Ethernet EE device stats & state
	NS_LOG_FUNCTION_NOARGS ();

	if (m_eee)
		{
			ChangeState(TX_SLEEP);
			ChangeState(RX_SLEEP);
			setTxLock(false);
			setRxLock(false);
			isCounting = false;
		}
	else
		{
			ChangeState(TX_READY);
			ChangeState(RX_READY);
			setTxLock(true);
			setRxLock(true);
		}

	m_lastTxUpdateTime = Simulator::Now ();
	m_lastRxUpdateTime = Simulator::Now ();

	t_tx_ready = 0;
	t_tx_busy = 0;
	t_tx_sleep = 0;
	t_tx_up = 0;
	t_tx_down = 0;
	t_rx_ready = 0;
	t_rx_busy = 0;
	t_rx_sleep = 0;
	t_rx_up = 0;
	t_rx_down = 0;

	m_totalTxEnergyConsumption = 0;
	m_totalTxEnergyConsumption_noEEE = 0;
	m_totalRxEnergyConsumption = 0;
	m_totalRxEnergyConsumption_noEEE = 0;

	m_sentPackets = 0;
	m_droppedPackets = 0;
	m_receivedPackets = 0;
	m_receivingError = 0;
	
	event_queue = 0;
	event_timeout = 0;

	m_currentPkt = 0;

}

PointToPointNetDevice::~PointToPointNetDevice ()
{
	NS_LOG_FUNCTION_NOARGS ();
}

/* ***************************************************************************************************
 *																		 CHANGE STATE
 * ***************************************************************************************************/

void
PointToPointNetDevice::ChangeState (RxMachineState newState)
{
	// Get duration
	Time duration = (Simulator::Now ()) - m_lastRxUpdateTime;
	NS_ASSERT (duration.GetNanoSeconds () >= 0);

	// Switch the current device state
	// Energy = current * voltage * time
	double energyConsumption = 0.0;
	double energyConsumption_noEEE = 0.0;

	switch (m_rxMachineState)
		{

			// Previous state was RX_READY
			case PointToPointNetDevice::RX_READY:
				{
					energyConsumption = duration.GetSeconds () * m_idlePower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_rx_ready += duration.GetSeconds ();
				}
				break;

			// Previous state was RX_BUSY
			case PointToPointNetDevice::RX_BUSY:
				{
					energyConsumption = duration.GetSeconds () * m_rxPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_rxPower;
					t_rx_busy += duration.GetSeconds ();
				}
				break;

			// Previous state was RX_UP
			case PointToPointNetDevice::RX_UP:
				{
					energyConsumption = duration.GetSeconds () * m_rxPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_rx_up += duration.GetSeconds ();
				}
				break;

			// Previous state was RX_DOWN
			case PointToPointNetDevice::RX_DOWN:
				{
					energyConsumption = duration.GetSeconds () * m_rxPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_rx_down += duration.GetSeconds ();
				}
				break;

			// Previous state was RX_SLEEP
			case PointToPointNetDevice::RX_SLEEP:
				{
					energyConsumption = duration.GetSeconds () * m_sleepPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_rx_sleep += duration.GetSeconds ();
				}
				break;

			// Error
			default:
				{
					NS_FATAL_ERROR ("PointToPointNetDevice:Undefined state: " << m_rxMachineState);
				}

		}

	// Update total energy consumption
	m_totalRxEnergyConsumption += energyConsumption;
	m_totalRxEnergyConsumption_noEEE += energyConsumption_noEEE;

	if (m_rxMachineState != newState)
		{
			switch (newState)
				{
					case PointToPointNetDevice::RX_SLEEP:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "RX_SLEEP" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::RX_READY:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "RX_READY" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::RX_BUSY:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "RX_BUSY" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::RX_UP:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "RX_UP" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::RX_DOWN:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "RX_DOWN" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					// Error
					default:
						{
							NS_FATAL_ERROR ("PointToPointNetDevice:Undefined state: " << m_rxMachineState);
						}
				}
		}

	// Update current state and last update time stamp
	m_lastRxUpdateTime = Simulator::Now ();
	m_rxMachineState = newState;

}

void
PointToPointNetDevice::ChangeState (TxMachineState newState)
{
	// Get duration
	Time duration = (Simulator::Now ()) - m_lastTxUpdateTime;
	NS_ASSERT (duration.GetNanoSeconds () >= 0);

	// Switch the current device state
	// Energy = current * voltage * time
	double energyConsumption = 0.0;
	double energyConsumption_noEEE = 0.0;

	switch (m_txMachineState)
		{

			// Previous state was TX_READY
			case PointToPointNetDevice::TX_READY:
				{
					energyConsumption = duration.GetSeconds () * m_idlePower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_tx_ready += duration.GetSeconds ();
				}
				break;

			// Previous state was TX_BUSY
			case PointToPointNetDevice::TX_BUSY:
				{
					energyConsumption = duration.GetSeconds () * m_txPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_txPower;
					t_tx_busy += duration.GetSeconds ();
				}
				break;

			// Previous state was TX_UP
			case PointToPointNetDevice::TX_UP:
				{
					energyConsumption = duration.GetSeconds () * m_txPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_tx_up += duration.GetSeconds ();
				}
				break;

			// Previous state was TX_DOWN
			case PointToPointNetDevice::TX_DOWN:
				{
					energyConsumption = duration.GetSeconds () * m_idlePower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_tx_down += duration.GetSeconds ();
				}
				break;

			// Previous state was TX_SLEEP
			case PointToPointNetDevice::TX_SLEEP:
				{
					energyConsumption = duration.GetSeconds () * m_sleepPower;
					energyConsumption_noEEE = duration.GetSeconds () * m_idlePower;
					t_tx_sleep += duration.GetSeconds ();
				}
				break;

			// Error
			default:
				{
					NS_FATAL_ERROR ("PointToPointNetDevice:Undefined state: " << m_txMachineState);
				}

		}

	// Update total energy consumption
	m_totalTxEnergyConsumption += energyConsumption;
	m_totalTxEnergyConsumption_noEEE += energyConsumption_noEEE;

	if (m_txMachineState != newState)
		{

			switch (newState)
				{
					case PointToPointNetDevice::TX_SLEEP:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "TX_SLEEP" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::TX_READY:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "TX_READY" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::TX_BUSY:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "TX_BUSY" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::TX_UP:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "TX_UP" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					case PointToPointNetDevice::TX_DOWN:
						{
							NS_LOG_INFO (m_address << "(" << TxLock << ", " << RxLock << ") [M] " 
										<< "\t" << " Device mode changed to " << "TX_DOWN" 
										<< " (+" << energyConsumption << "J during " << duration << ")");
						}
						break;
					// Error
					default:
						{
							NS_FATAL_ERROR ("PointToPointNetDevice:Undefined state: " << m_txMachineState);
						}
				}

		}

	// Update current state and last update time stamp
	m_lastTxUpdateTime = Simulator::Now ();
	m_txMachineState = newState;

}

/* ***************************************************************************************************
 *															         			 TRANSMIT
 * ***************************************************************************************************/

bool
PointToPointNetDevice::Send (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
{

	NS_LOG_FUNCTION_NOARGS ();
	NS_LOG_LOGIC ("p=" << packet << ", dest=" << &dest);
	NS_LOG_LOGIC ("UID is " << packet->GetUid ());

	m_sentPackets++;

	// If IsLinkUp() is false it means there is no channel to send any packet
	// over so we just hit the drop trace on the packet and return an error.
	if (IsLinkUp () == false)
		{
			m_macTxDropTrace (packet);
			m_droppedPackets++;
			return false;
		}

	// Ethernet Encap
	EthernetHeader header;
	header.SetSource (m_address);
	header.SetDestination (Mac48Address::ConvertFrom(dest));
	header.SetLengthType (protocolNumber);
	packet->AddHeader (header);
	EthernetTrailer trailer;
	trailer.CalcFcs (packet);
	packet->AddTrailer (trailer);

	// Trace
	m_macTxTrace (packet);

	if (m_queue->Enqueue (packet) == false)
		{
			NS_LOG_WARN (m_address << "(" << TxLock << ", " << RxLock << ") [!] " 
						<< "\t\t" << " Warning: failed to add packet to queue");
			m_macTxDropTrace (packet);
			m_droppedPackets++;
			return false;
		}
	else
	  	{
			NS_LOG_LOGIC (m_address << "(" << TxLock << ", " << RxLock << ") [+] " 
						<< "\t\t" << " Packet added (" << m_queue->GetNPackets () << ")");
			
			initTransmissionIfPossible();
		  	return true;
		}
}

bool
PointToPointNetDevice::SendFrom (Ptr<Packet> packet, const Address &source, const Address &dest, uint16_t protocolNumber)
{

	NS_LOG_FUNCTION_NOARGS ();
	NS_LOG_LOGIC ("p=" << packet << ", dest=" << &dest);
	NS_LOG_LOGIC ("UID is " << packet->GetUid ());

	// If IsLinkUp() is false it means there is no channel to send any packet
	// over so we just hit the drop trace on the packet and return an error.
	if (IsLinkUp () == false)
		{
			m_macTxDropTrace (packet);
			return false;
		}

	// Ethernet Encap
	if (Mac48Address::ConvertFrom(source) == m_address)
	{
		EthernetHeader header;
		header.SetSource (Mac48Address::ConvertFrom(source));
		header.SetDestination (Mac48Address::ConvertFrom(dest));
		header.SetLengthType (protocolNumber);
		packet->AddHeader (header);
		EthernetTrailer trailer;
		trailer.CalcFcs (packet);
		packet->AddTrailer (trailer);
	}

	// Trace
	m_macTxTrace (packet);

	if (m_queue->Enqueue (packet) == false)
		{
			NS_LOG_WARN (m_address << "(" << TxLock << ", " << RxLock << ") [!] " 
						<< "\t\t" << " Warning: failed to add packet to queue");
			m_macTxDropTrace (packet);
			m_droppedPackets++;
			return false;
		}
	else
		{
			NS_LOG_LOGIC (m_address << "(" << TxLock << ", " << RxLock << ") [+] " 
						<< "\t\t" << " Packet added (" << m_queue->GetNPackets () << ")");
			
			initTransmissionIfPossible();
		  	return true;
		}

}

void
PointToPointNetDevice::initTransmissionIfPossible ()
{
	if (m_queue->GetNPackets() == 0) 
		return;
	
	switch (m_txMachineState)
	{
		case TX_READY:
			initTransmission (); 
			break;
		case TX_BUSY: case TX_UP:
			break;
		case TX_SLEEP: case TX_DOWN:
			{
			  // Check queue threshold event
			  UintegerValue m_queue_capacity;
			  m_queue->GetAttribute ("MaxPackets", m_queue_capacity);
			  unsigned int queue_threshold = (unsigned int) ((double) ((double) m_queue_factor / (double) 100) * (double) m_queue_capacity.Get());
			    
			  if (m_queue->GetNPackets () > queue_threshold)
				{
				  // Max queue event
				  event_queue++;
				  NS_LOG_DEBUG (m_address << "(" << TxLock << ", " << RxLock << ") [!] " 
									<< "\t" << " (full    - " << m_queue->GetNPackets () << "/" << queue_threshold << " packets - " 
									<< (m_ticks_max - m_ticks_count) << "/" << m_ticks_max << " ticks)");
				  
				  // Disable 1st packet timer and start transmission
				  if (isCounting)
					isCounting = false;
				
				  if (m_txMachineState == TX_SLEEP)
				    {
					  // Wake up the connection
					  NIC_UP ();
					  SendPCSMessage(PointToPointNetDevice::PCS_UP);
				    }
				  if (m_txMachineState == TX_DOWN)
					{
					  ChangeState (TX_READY);
					  ChangeState (RX_READY);
					  initTransmission();  
				    }
				}
			  else if (isCounting == false)
			    {
				  // Enable 1st packet timer out
				  isCounting = true;
				  Simulator::Schedule (Seconds (beta * m_t_up), &m_TxSleepClockTrigger, this);
			    }
			}
			break;
		default: NS_ASSERT(false);
	}
}

void
PointToPointNetDevice::initTransmission ()
{
	// Assert there is something to transmit
	NS_ASSERT (m_queue->GetNPackets() > 0);
	NS_LOG_FUNCTION_NOARGS ();
  
	// Tx Lock
	if (!TxLock)
		{
			setTxLock(true);
		}
	
	// Get 1st packet
	Ptr<Packet> packet = m_queue->Dequeue ();
  
	// Start sniffer
	m_snifferTrace (packet);
	m_promiscSnifferTrace (packet);

	NS_ASSERT (m_txMachineState == TX_READY);
	// Start transmission
	if (m_txMachineState == TX_READY)
		{
			// Start transmitting the first packet
			TransmitStart (packet);
		}
}

bool
PointToPointNetDevice::TransmitStart (Ptr<Packet> p)
{

	NS_LOG_INFO (m_address << " >>>> Transmission start");
	NS_LOG_LOGIC ("UID is " << p->GetUid () << ")");


	// Change state to TX_BUSY
	NS_ASSERT_MSG (m_txMachineState == TX_READY, "Must be TX_READY to transmit");
	ChangeState (TX_BUSY);
	NS_ASSERT_MSG (m_txMachineState == TX_BUSY, "Must be TX_BUSY");

	// Set current packet
	m_currentPkt = p;
	m_phyTxBeginTrace (m_currentPkt);

	// Schedule TxComplete event
	Time txTime = Seconds (m_bps.CalculateTxTime (p->GetSize ()));
	Time txCompleteTime = txTime + m_tInterframeGap;
	NS_LOG_LOGIC ("Schedule TransmitCompleteEvent in " << txCompleteTime.GetSeconds () << "sec");
	Simulator::Schedule (txCompleteTime, &PointToPointNetDevice::TransmitComplete, this);

	// Send the packet
	bool result = m_channel->TransmitStart (p, this, txTime);

	if (result == false)
		{
			// Packet dropped
			m_phyTxDropTrace (p);
		}

	return result;
}

void
PointToPointNetDevice::TransmitComplete (void)
{
	NS_LOG_INFO (m_address << " #### Transmission complete");
	
	NS_LOG_LOGIC (m_address << "(" << TxLock << ", " << RxLock << ") [-] " 
					<< "\t\t" << " Packet removed (" << m_queue->GetNPackets() << ")");
	// This function is called to when we're all done transmitting a packet. We try and pull another packet off of the transmit queue.
	// If the queue is empty, we are done, otherwise we need to start transmitting the next packet.

	// Tx done
	NS_ASSERT (m_txMachineState == TX_BUSY);
	ChangeState (TX_READY);
	NS_ASSERT (m_txMachineState == TX_READY);

	NS_ASSERT_MSG (m_currentPkt != 0, "PointToPointNetDevice::TransmitComplete(): m_currentPkt zero");

	// Reset current packet
	m_phyTxEndTrace (m_currentPkt);
	m_currentPkt = 0;

	// Try to get next packet
	Ptr<Packet> p = m_queue->Dequeue ();

	if (p != 0)
		{
			// Got another packet off of the queue, so start the transmit process again
			m_snifferTrace (p);
			m_promiscSnifferTrace (p);
			TransmitStart (p);
		}
	else
		{
			// No packet was on the queue: Stop transmitting
			setTxLock(false);
			
			// Say to remote device: I wanna sleep.
			SendPCSMessage(PCS_DOWN);
			
			// Received PCS_DOWN in past?
			if (!RxLock)
				{
					// Received PCS_DOWN : Go Sleep	
					NIC_DOWN();
				}
		}
}

/* ***************************************************************************************************
 *															         			 RECEIVE
 * ***************************************************************************************************/
void
PointToPointNetDevice::ReceiveStart (void)
{
	if ((m_rxMachineState == RX_READY) || (m_rxMachineState == RX_DOWN))
		{
			m_receivedPackets++;
			ChangeState (RX_BUSY);
			if (!RxLock) 
			  {
				setRxLock(true);
			  }
			if (m_txMachineState == TX_DOWN)
				ChangeState (TX_READY);
		}
	else 
		{
			// Error message
			m_receivingError++;
		}
	return;
}

void
PointToPointNetDevice::Receive (Ptr<Packet> packet)
{
	NS_LOG_INFO (m_address << " \t\t\t\tPacket has been fully received");
	uint16_t protocol = 0;
	
	// Local device starts to receive packets while being transmitting
	if ((m_rxMachineState == RX_BUSY) || (m_rxMachineState == RX_DOWN))
		{
			
			if (m_rxMachineState == RX_BUSY)
				{
					// Add Rx busy time in ChangeState
					ChangeState (RX_READY);
				}
			else
				{
					// Device is waiting to sleep: Add the Rx busy time
					Time rxTime = Seconds (m_bps.CalculateTxTime (packet->GetSize ()));
					Time rxCompleteTime = rxTime + m_tInterframeGap;
					t_rx_busy += rxTime.GetSeconds ();
				}
    
			if (m_receiveErrorModel && m_receiveErrorModel->IsCorrupt (packet))
				{
					// If we have an error model and it indicates that it is time to lose a
					// corrupted packet, don't forward this packet up, let it go
					m_phyRxDropTrace (packet);
				}
			else
				{
					// Hit the trace hooks. All of these hooks are in the same place in this
					// device becuase it is so simple, but this is not usually the case in
					// more complicated devices
					m_snifferTrace (packet);
					m_promiscSnifferTrace (packet);
					m_phyRxEndTrace (packet);

					// Ethernet Decap
					if (m_bridge)
						{
							// Just read source and destination, and forward
							EthernetHeader header;
							packet->PeekHeader (header);
							protocol = header.GetLengthType();
							Mac48Address addr_source = header.GetSource();
							Mac48Address addr_destination = header.GetDestination();
							EthernetTrailer trailer;
							packet->PeekTrailer (trailer);

							if (!m_promiscCallback.IsNull ())
								{
									// Promiscuous callback
									m_macPromiscRxTrace (packet);
									m_promiscCallback (this, packet, protocol, addr_source, addr_destination, NetDevice::PACKET_OTHERHOST);
								}
						}
					else
						{
							// Remove Ethernet header and trailer
							EthernetHeader header;
							packet->RemoveHeader (header);
							protocol = header.GetLengthType();
							Mac48Address addr_source = header.GetSource();
							Mac48Address addr_destination = header.GetDestination();
							EthernetTrailer trailer;
							packet->RemoveTrailer (trailer);

							if (!m_promiscCallback.IsNull ())
								{
									// Promiscuous callback
									m_macPromiscRxTrace (packet);
									m_promiscCallback (this, packet, protocol, addr_source, addr_destination, NetDevice::PACKET_HOST);
								}

							// Standard callback
							m_macRxTrace (packet);
							m_rxCallback (this, packet, protocol, addr_source);
						}

				}
    
		}
	else
		{
			NS_LOG_WARN (m_address << "(" << TxLock << ", " << RxLock << ") [D] " << Simulator::Now ().GetSeconds() << "\t" << " Message fully received while not RX_BUSY (" << RX_READY << ") or RX_DOWN (" << RX_DOWN << ") : " << m_rxMachineState);
			ChangeState (RX_READY);
			m_receivingError++;
		}

}

/* ***************************************************************************************************
 *																		 OTHER
 * ***************************************************************************************************/

void
PointToPointNetDevice::DisableEEE ()
{
	NS_LOG_FUNCTION (this);
	m_eee = false;
	m_txMachineState = TX_READY;
	m_rxMachineState = RX_READY;
  setTxLock(true);
  setRxLock(true);
}

void
PointToPointNetDevice::SetBridgeMode (bool mode)
{
	NS_LOG_FUNCTION (this << mode);
	m_bridge = mode;
}

bool
PointToPointNetDevice::Attach (Ptr<PointToPointChannel> ch)
{
	NS_LOG_FUNCTION (this << &ch);

	m_channel = ch;
	m_channel->Attach (this);

	NotifyLinkUp ();
	return true;
}

void
PointToPointNetDevice::SetQueue (Ptr<Queue> q)
{
	NS_LOG_FUNCTION (this << q);
	m_queue = q;
	m_queue->SetAttribute ("MaxPackets", UintegerValue(m_packets_max));
}

void
PointToPointNetDevice::SetReceiveErrorModel (Ptr<ErrorModel> em)
{
	NS_LOG_FUNCTION (this << em);
	m_receiveErrorModel = em;
}

Ptr<Queue>
PointToPointNetDevice::GetQueue (void) const
{
	NS_LOG_FUNCTION_NOARGS ();
	return m_queue;
}

void
PointToPointNetDevice::NotifyLinkUp (void)
{
	m_linkUp = true;
	m_linkChangeCallbacks ();
}

void
PointToPointNetDevice::SetIfIndex (const uint32_t index)
{
	m_ifIndex = index;
}

uint32_t
PointToPointNetDevice::GetIfIndex (void) const
{
	return m_ifIndex;
}

Ptr<Channel>
PointToPointNetDevice::GetChannel (void) const
{
	return m_channel;
}

void
PointToPointNetDevice::SetAddress (Address address)
{
	m_address = Mac48Address::ConvertFrom (address);
}

Address
PointToPointNetDevice::GetAddress (void) const
{
	return m_address;
}

bool
PointToPointNetDevice::IsLinkUp (void) const
{
	return m_linkUp;
}

void
PointToPointNetDevice::AddLinkChangeCallback (Callback<void> callback)
{
	m_linkChangeCallbacks.ConnectWithoutContext (callback);
}

bool
PointToPointNetDevice::IsBroadcast (void) const
{
	return true;
}

Address
PointToPointNetDevice::GetBroadcast (void) const
{
	return Mac48Address ("ff:ff:ff:ff:ff:ff");
}

bool
PointToPointNetDevice::IsMulticast (void) const
{
	return true;
}

Address
PointToPointNetDevice::GetMulticast (Ipv4Address multicastGroup) const
{
	return Mac48Address ("01:00:5e:00:00:00");
}

Address
PointToPointNetDevice::GetMulticast (Ipv6Address addr) const
{
	NS_LOG_FUNCTION (this << addr);
	return Mac48Address ("33:33:00:00:00:00");
}

bool
PointToPointNetDevice::IsPointToPoint (void) const
{
	return true;
}

bool
PointToPointNetDevice::IsBridge (void) const
{
	return false;
}

void
PointToPointNetDevice::DoDispose ()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_node = 0;
	m_channel = 0;
	m_receiveErrorModel = 0;
	m_currentPkt = 0;
	NetDevice::DoDispose ();
}

Ptr<Node>
PointToPointNetDevice::GetNode (void) const
{
	return m_node;
}

void
PointToPointNetDevice::SetNode (Ptr<Node> node)
{
	m_node = node;
}

bool
PointToPointNetDevice::NeedsArp (void) const
{
	return true;
}

void
PointToPointNetDevice::SetReceiveCallback (NetDevice::ReceiveCallback cb)
{
	m_rxCallback = cb;
}

void
PointToPointNetDevice::SetPromiscReceiveCallback (NetDevice::PromiscReceiveCallback cb)
{
	m_promiscCallback = cb;
}

bool
PointToPointNetDevice::SupportsSendFrom (void) const
{
	return true;
}

void
PointToPointNetDevice::DoMpiReceive (Ptr<Packet> p)
{
	Receive (p);
}

void
PointToPointNetDevice::doEnergyRefresh ()
{
	ChangeState (m_txMachineState);
	ChangeState (m_rxMachineState);
}

void
PointToPointNetDevice::SetDataRate (DataRate bps)
{
	NS_LOG_FUNCTION_NOARGS ();
	m_bps = bps;
}

void
PointToPointNetDevice::SetInterframeGap (Time t)
{
	NS_LOG_FUNCTION_NOARGS ();
	m_tInterframeGap = t;
}

double
PointToPointNetDevice::getTotalTxEnergyConsumption ()
{
	return m_totalTxEnergyConsumption;
}

double
PointToPointNetDevice::getTotalTxEnergyConsumptionNoEEE ()
{
	return m_totalTxEnergyConsumption_noEEE;
}

double
PointToPointNetDevice::getTotalRxEnergyConsumption ()
{
	return m_totalRxEnergyConsumption;
}

double
PointToPointNetDevice::getTotalRxEnergyConsumptionNoEEE ()
{
	return m_totalRxEnergyConsumption_noEEE;
}

enum PointToPointNetDevice::RxMachineState
PointToPointNetDevice::getRxState ()
{
	return m_rxMachineState;
}

enum PointToPointNetDevice::TxMachineState
PointToPointNetDevice::getTxState ()
{
	return m_txMachineState;
}

double
PointToPointNetDevice::getTxSleepTime ()
{
	return t_tx_sleep;
}

double
PointToPointNetDevice::getTxBusyTime ()
{
	return t_tx_busy;
}

double
PointToPointNetDevice::getTxUpTime ()
{
	return t_tx_up;
}

double
PointToPointNetDevice::getTxDownTime ()
{
	return t_tx_down;
}

double
PointToPointNetDevice::getTxReadyTime ()
{
	return t_tx_ready;
}

double
PointToPointNetDevice::getRxSleepTime ()
{
	return t_rx_sleep;
}

double
PointToPointNetDevice::getRxBusyTime ()
{
	return t_rx_busy;
}

double
PointToPointNetDevice::getRxUpTime ()
{
	return t_rx_up;
}

double
PointToPointNetDevice::getRxDownTime ()
{
	return t_rx_down;
}

double
PointToPointNetDevice::getRxReadyTime ()
{
	return t_rx_ready;
}

double
PointToPointNetDevice::getSentPackets() 
{
	return m_sentPackets;
}

double
PointToPointNetDevice::getDroppedPackets ()
{
	return m_droppedPackets;
}

double
PointToPointNetDevice::getReceivedPackets ()
{
	return m_receivedPackets;
}

double
PointToPointNetDevice::getReceivingError ()
{
	return m_receivingError;
}

Time
PointToPointNetDevice::getMaxQueueTime ()
{
	return Seconds (m_ticks_max * beta * m_t_up);
}

double
PointToPointNetDevice::getEventQueueRatio ()
{
	if ((event_queue + event_timeout) != 0)
		{
			return (double) 100 * ((double) event_queue / (double) (event_queue + event_timeout));
		}
	else
		{
			return 0;
		}
}

double
PointToPointNetDevice::getEventTimeoutRatio ()
{
	if ((event_queue + event_timeout) != 0)
		{
			return (double) 100 * ((double) event_timeout / (double) (event_queue + event_timeout));
		}
	else
		{
			return 0;
		}
}

Address
PointToPointNetDevice::GetRemote (void) const
{
	NS_ASSERT (m_channel->GetNDevices () == 2);
	for (uint32_t i = 0; i < m_channel->GetNDevices (); ++i)
		{
			Ptr<NetDevice> tmp = m_channel->GetDevice (i);
			if (tmp != this)
				{
					return tmp->GetAddress ();
				}
		}
	NS_ASSERT (false);
	return Address ();
}

bool
PointToPointNetDevice::SetMtu (uint16_t mtu)
{
	NS_LOG_FUNCTION (this << mtu);
	m_mtu = mtu;
	return true;
}

uint16_t
PointToPointNetDevice::GetMtu (void) const
{
	NS_LOG_FUNCTION_NOARGS ();
	return m_mtu;
}

TypeId
PointToPointNetDevice::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::PointToPointNetDevice")

		.SetParent<NetDevice> ()
		.AddConstructor<PointToPointNetDevice> ()

		// Attributes
		.AddAttribute ("MaxTicks", "Maximum number of clock ticks for a packet in queue",
									 UintegerValue (150),
									 MakeUintegerAccessor (&PointToPointNetDevice::m_ticks_max),
									 MakeUintegerChecker<uint16_t> ())
		.AddAttribute ("MaxPackets", "Maximum number of packets in queue",
									 UintegerValue (300),
									 MakeUintegerAccessor (&PointToPointNetDevice::m_packets_max),
									 MakeUintegerChecker<uint16_t> ())
		.AddAttribute ("QueueFactor", "Queue factor",
									 UintegerValue (10),
									 MakeUintegerAccessor (&PointToPointNetDevice::m_queue_factor),
									 MakeUintegerChecker<uint16_t> ())
		.AddAttribute ("Mtu", "The MAC-level Maximum Transmission Unit",
									 UintegerValue (DEFAULT_MTU),
									 MakeUintegerAccessor (&PointToPointNetDevice::SetMtu, &PointToPointNetDevice::GetMtu),
									 MakeUintegerChecker<uint16_t> ())
		.AddAttribute ("Address",
									 "The MAC address of this device.",
									 Mac48AddressValue (Mac48Address ("ff:ff:ff:ff:ff:ff")),
									 MakeMac48AddressAccessor (&PointToPointNetDevice::m_address),
									 MakeMac48AddressChecker ())
		.AddAttribute ("DataRate",
									 "The default data rate for point to point links",
									 DataRateValue (DataRate ("1Gbps")),
									 MakeDataRateAccessor (&PointToPointNetDevice::m_bps),
									 MakeDataRateChecker ())
		.AddAttribute ("ReceiveErrorModel",
									 "The receiver error model used to simulate packet loss",
									 PointerValue (),
									 MakePointerAccessor (&PointToPointNetDevice::m_receiveErrorModel),
									 MakePointerChecker<ErrorModel> ())
		.AddAttribute ("InterframeGap",
									 "The time to wait between packet (frame) transmissions",
									 TimeValue (Seconds (0.0)),
									 MakeTimeAccessor (&PointToPointNetDevice::m_tInterframeGap),
									 MakeTimeChecker ())
		.AddAttribute ("TxQueue",
									 "A queue to use as the transmit queue in the device.",
									 PointerValue (),
									 MakePointerAccessor (&PointToPointNetDevice::m_queue),
									 MakePointerChecker<Queue> ())

		// Trace
		.AddTraceSource ("MacTx",
										 "Trace source indicating a packet has arrived for transmission by this device",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_macTxTrace))
		.AddTraceSource ("MacTxDrop",
										 "Trace source indicating a packet has been dropped by the device before transmission",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_macTxDropTrace))
		.AddTraceSource ("MacPromiscRx",
										 "A packet has been received by this device, has been passed up from the physical layer "
										 "and is being forwarded up the local protocol stack.	This is a promiscuous trace,",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_macPromiscRxTrace))
		.AddTraceSource ("MacRx",
										 "A packet has been received by this device, has been passed up from the physical layer "
										 "and is being forwarded up the local protocol stack.	This is a non-promiscuous trace,",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_macRxTrace))
		.AddTraceSource ("PhyTxBegin",
										 "Trace source indicating a packet has begun transmitting over the channel",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_phyTxBeginTrace))
		.AddTraceSource ("PhyTxEnd",
										 "Trace source indicating a packet has been completely transmitted over the channel",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_phyTxEndTrace))
		.AddTraceSource ("PhyTxDrop",
										 "Trace source indicating a packet has been dropped by the device during transmission",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_phyTxDropTrace))
		.AddTraceSource ("PhyRxEnd",
										 "Trace source indicating a packet has been completely received by the device",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_phyRxEndTrace))
		.AddTraceSource ("PhyRxDrop",
										 "Trace source indicating a packet has been dropped by the device during reception",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_phyRxDropTrace))
		.AddTraceSource ("Sniffer",
										 "Trace source simulating a non-promiscuous packet sniffer attached to the device",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_snifferTrace))
		.AddTraceSource ("PromiscSniffer",
										 "Trace source simulating a promiscuous packet sniffer attached to the device",
										 MakeTraceSourceAccessor (&PointToPointNetDevice::m_promiscSnifferTrace))
	;
	return tid;
}

} // namespace ns3
