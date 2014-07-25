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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef POINT_TO_POINT_NET_DEVICE_H
#define POINT_TO_POINT_NET_DEVICE_H

#include <string.h>
#include "ns3/address.h"
#include "ns3/node.h"
#include "ns3/net-device.h"
#include "ns3/callback.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/ptr.h"
#include "ns3/mac48-address.h"
#include "ns3/traced-value.h"
#include "ns3/timer.h"

namespace ns3 {

class Queue;
class PointToPointChannel;
class ErrorModel;

/**
 * \defgroup point-to-point PointToPointNetDevice
 * This section documents the API of the ns-3 point-to-point module. For a generic functional description, please refer to the ns-3 manual.
 */

/**
 * \ingroup point-to-point
 * \class PointToPointNetDevice
 * \brief A Device for a Point to Point Network Link.
 *
 * This PointToPointNetDevice class specializes the NetDevice abstract
 * base class.  Together with a PointToPointChannel (and a peer
 * PointToPointNetDevice), the class models, with some level of
 * abstraction, a generic point-to-point or serial link.
 * Key parameters or objects that can be specified for this device
 * include a queue, data rate, and interframe transmission gap (the
 * propagation delay is set in the PointToPointChannel).
 */
class PointToPointNetDevice : public NetDevice
{
public:

  static TypeId GetTypeId (void);

  /**
   * Construct a PointToPointNetDevice
   *
   * This is the constructor for the PointToPointNetDevice.  It takes as a
   * parameter a pointer to the Node to which this device is connected,
   * as well as an optional DataRate object.
   */
  PointToPointNetDevice ();

  /**
   * Destroy a PointToPointNetDevice
   *
   * This is the destructor for the PointToPointNetDevice.
   */
  virtual ~PointToPointNetDevice ();

  /**
   * Set the Data Rate used for transmission of packets.  The data rate is
   * set in the Attach () method from the corresponding field in the channel
   * to which the device is attached.  It can be overridden using this method.
   *
   * @see Attach ()
   * @param bps the data rate at which this object operates
   */
  void SetDataRate (DataRate bps);

  /**
   * Set the interframe gap used to separate packets.  The interframe gap
   * defines the minimum space required between packets sent by this device.
   *
   * @param t the interframe gap time
   */
  void SetInterframeGap (Time t);

  /**
   * Attach the device to a channel.
   *
   * @param ch Ptr to the channel to which this object is being attached.
   */
  bool Attach (Ptr<PointToPointChannel> ch);

  /**
   * Attach a queue to the PointToPointNetDevice.
   *
   * The PointToPointNetDevice "owns" a queue that implements a queueing
   * method such as DropTail or RED.
   *
   * @see Queue
   * @see DropTailQueue
   * @param queue Ptr to the new queue.
   */
  void SetQueue (Ptr<Queue> queue);

  /**
   * Get a copy of the attached Queue.
   *
   * @returns Ptr to the queue.
   */
  Ptr<Queue> GetQueue (void) const;

  /**
   * Attach a receive ErrorModel to the PointToPointNetDevice.
   *
   * The PointToPointNetDevice may optionally include an ErrorModel in
   * the packet receive chain.
   *
   * @see ErrorModel
   * @param em Ptr to the ErrorModel.
   */
  void SetReceiveErrorModel (Ptr<ErrorModel> em);

  /**
   * Receive a packet from a connected PointToPointChannel.
   *
   * The PointToPointNetDevice receives packets from its connected channel
   * and forwards them up the protocol stack.  This is the public method
   * used by the channel to indicate that the last bit of a packet has
   * arrived at the device.
   *
   * @see PointToPointChannel
   * @param p Ptr to the received packet.
   */
  void Receive (Ptr<Packet> p);

  // The remaining methods are documented in ns3::NetDevice*

  virtual void SetIfIndex (const uint32_t index);
  virtual uint32_t GetIfIndex (void) const;

  virtual Ptr<Channel> GetChannel (void) const;

  virtual void SetAddress (Address address);
  virtual Address GetAddress (void) const;

  virtual bool SetMtu (const uint16_t mtu);
  virtual uint16_t GetMtu (void) const;

  virtual bool IsLinkUp (void) const;

  virtual void AddLinkChangeCallback (Callback<void> callback);

  virtual bool IsBroadcast (void) const;
  virtual Address GetBroadcast (void) const;

  virtual bool IsMulticast (void) const;
  virtual Address GetMulticast (Ipv4Address multicastGroup) const;

  virtual bool IsPointToPoint (void) const;
  virtual bool IsBridge (void) const;

  virtual bool Send (Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber);
  virtual bool SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber);

  virtual Ptr<Node> GetNode (void) const;
  virtual void SetNode (Ptr<Node> node);

  virtual bool NeedsArp (void) const;

  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);

  virtual Address GetMulticast (Ipv6Address addr) const;

  virtual void SetPromiscReceiveCallback (PromiscReceiveCallback cb);
  virtual bool SupportsSendFrom (void) const;

  /**
   * Added public functions
   */
  double getTotalTxEnergyConsumption();
  double getTotalTxEnergyConsumptionNoEEE();
  double getTotalRxEnergyConsumption();
  double getTotalRxEnergyConsumptionNoEEE();

  double getTxSleepTime();
  double getTxBusyTime();
  double getTxUpTime();
  double getTxDownTime();
  double getTxReadyTime();

  double getRxSleepTime();
  double getRxBusyTime();
  double getRxUpTime();
  double getRxDownTime();
  double getRxReadyTime();

  double getEventQueueRatio();
  double getEventTimeoutRatio();

  Time getMaxQueueTime();         // Get tmax
  bool IsLocalReceiveOn();        // Sense line
  
  void doEnergyRefresh();         // Update energy cons
  void m_TxSleepClock();          // Sleep clock period
  void SetBridgeMode (bool mode); // Bridge mode
  void DisableEEE();              // Disable Energy-Efficiency
  void Reset();                   // Reset

  void m_TxWakeUp();              // Tx Up
  void m_RxWakeUp();              // Rx Up
  void m_TxPowerDown();           // Tx Down
  void m_RxPowerDown();           // Rx Down
  
  /**
   * PCS emulation
   */
  enum PCSMessage
    {
      PCS_UP,   /**< PCS wake up message */
      PCS_DOWN  /**< PCS sleep message */
    };

  // High & low level PCS functions
  void SendPCSMessage (PCSMessage msg);
  void SendPCSMessageNow (PCSMessage msg);
  void ReceivePCSMessage (PCSMessage msg);
  
  Ptr<PointToPointNetDevice> GetRemoteNetDevice (void) const;

private:

  // Start Tx queue processing
  void initTransmission();
  void initTransmissionIfPossible();
  
  // Wake up and sleep functions
  void NIC__TX__UP();
  void NIC__RX__UP();
  void NIC__TX__DOWN();
  void NIC__RX__DOWN();

  void NIC__UP ();
  void NIC__DOWN ();

  // Beta parameter
  double beta;

  /**
   * Enumeration of the states of the transmit machine of the net device.
   */
  enum TxMachineState
    {
      TX_READY,      /**< 0: The transmitter is ready to begin transmission of a packet */
      TX_BUSY,       /**< 1: The transmitter is busy transmitting a packet */
      TX_SLEEP,      /**< 2: The transmitter is sleeping */
      TX_UP,         /**< 3: The transmitter is powering on */
      TX_DOWN        /**< 4: The transmitter is powering off */
    };

  enum RxMachineState
    {
      RX_READY,      /**< 0: The receiver is ready to receive a packet */
      RX_BUSY,       /**< 1: The receiver is busy receiving a packet */
      RX_SLEEP,      /**< 2: The receiver is sleeping */
      RX_UP,         /**< 3: The receiver is powering on */
      RX_DOWN        /**< 4: The receiver is powering off */
    };

  void ChangeState (TxMachineState newState);
  void ChangeState (RxMachineState newState);

  enum TxMachineState getTxState ();
  enum RxMachineState getRxState ();

  bool TxLock;
  bool RxLock;
  
  void setTxLock(bool b);
  void setRxLock(bool b);

  PointToPointNetDevice& operator = (const PointToPointNetDevice &);
  PointToPointNetDevice (const PointToPointNetDevice &);

  virtual void DoDispose (void);

private:

  /**
   * The state of the Net Device transmit state machine.
   * @see TxMachineState
   */
  TxMachineState m_txMachineState;
  RxMachineState m_rxMachineState;

  Time m_lastTxUpdateTime;
  Time m_lastRxUpdateTime;

  /**
   * Current values when datarate = 1 Gbps and packet size = 1000 Bytes
   */
  // PCI express interface
  static const double m_idlePower = 1;                // TX_READY, RX_READY
  static const double m_sleepPower = 0.1;             // TX_SLEEP, RX_SLEEP
  static const double m_txPower = 2;                  // TX_BUSY, TX_UP, TX_DOWN
  static const double m_rxPower = 1.3;                // RX_BUSY, RX_UP, RX_DOWN
  // Switch interface
  //static const double m_idlePower = 0.175;            // TX_READY, RX_READY
  //static const double m_sleepPower = 0.019375;        // TX_SLEEP, RX_SLEEP
  //static const double m_txPower = 0.1875;             // TX_BUSY, TX_UP, TX_DOWN
  //static const double m_rxPower = 0.175;              // RX_BUSY, RX_UP, RX_DOWN
  
  /**
   * Wake and sleep time parameters
   */
  static const double m_t_up = 0.0000165;             // 16.5 micro-seconds = 0.0000165 (page 254/264 IEEE 802.3-az 2010 1000BASE-T)
  static const double m_t_down = 0.0001820;           // 182 micro-seconds = 0.0001820 (page 246/264 IEEE 802.3-az 2010 1000BASE-T)

  /**
   * Tx start parameters
   */
  unsigned int m_queue_factor;                        // Queue capacity (in packets) : b = m_queue_factor * B
  unsigned int m_ticks_max;                           // Maximum number of clock ticks (2.50 milli-seconds = 0.0025 recommended)
  unsigned int m_packets_max;                         // Maximum number of packets in queue
  int m_ticks_count;                                  // Number of clock ticks

  /**
   * Events count
   */
  unsigned int event_queue;
  unsigned int event_timeout;

  /**
   * Interface parameters
   */
  bool m_bridge;                                      // Bridge mode activated
  bool m_eee;                                         // EEE enabled
  
  /**
   * Traced values
   */

  // Energy consumption
  TracedValue<double> m_totalTxEnergyConsumption;
  TracedValue<double> m_totalTxEnergyConsumption_noEEE;
  TracedValue<double> m_totalRxEnergyConsumption;
  TracedValue<double> m_totalRxEnergyConsumption_noEEE;

  // Packet stats
  TracedValue<double> m_packetsTotal;
  TracedValue<double> m_packetsDropped;

  // Tx times
  TracedValue<double> t_tx_ready;
  TracedValue<double> t_tx_busy;
  TracedValue<double> t_tx_sleep;
  TracedValue<double> t_tx_up;
  TracedValue<double> t_tx_down;

  // Rx times
  TracedValue<double> t_rx_ready;
  TracedValue<double> t_rx_busy;
  TracedValue<double> t_rx_sleep;
  TracedValue<double> t_rx_up;
  TracedValue<double> t_rx_down;

  /**
   * \returns the address of the remote device connected to this device
   * through the point to point channel.
   */
  Address GetRemote (void) const;

  /**
   * Adds the necessary headers and trailers to a packet of data in order to
   * respect the protocol implemented by the agent.
   * \param p packet
   * \param protocolNumber protocol number
   */
  void EthernetEncap (Ptr<Packet> p, uint16_t protocolNumber, Mac48Address m_source, Mac48Address m_destination);

  /**
   * Removes, from a packet of data, all headers and trailers that
   * relate to the protocol implemented by the agent
   * \param p Packet whose headers need to be processed
   * \param param An integer parameter that can be set by the function
   * \return Returns true if the packet should be forwarded up the
   * protocol stack.
   */
  bool EthernetDecap (Ptr<Packet> p, uint16_t& param);

  /**
   * Start Sending a Packet Down the Wire.
   *
   * The TransmitStart method is the method that is used internally in the
   * PointToPointNetDevice to begin the process of sending a packet out on
   * the channel.  The corresponding method is called on the channel to let
   * it know that the physical device this class represents has virtually
   * started sending signals.  An event is scheduled for the time at which
   * the bits have been completely transmitted.
   *
   * @see PointToPointChannel::TransmitStart ()
   * @see TransmitCompleteEvent ()
   * @param p a reference to the packet to send
   * @returns true if success, false on failure
   */
  bool TransmitStart (Ptr<Packet> p);

  /**
   * Stop Sending a Packet Down the Wire and Begin the Interframe Gap.
   *
   * The TransmitComplete method is used internally to finish the process
   * of sending a packet out on the channel.
   */
  void TransmitComplete (void);

  void NotifyLinkUp (void);

  /**
   * The data rate that the Net Device uses to simulate packet transmission
   * timing.
   * @see class DataRate
   */
  DataRate m_bps;

  /**
   * The interframe gap that the Net Device uses to throttle packet
   * transmission
   * @see class Time
   */
  Time m_tInterframeGap;

  /**
   * The PointToPointChannel to which this PointToPointNetDevice has been
   * attached.
   * @see class PointToPointChannel
   */
  Ptr<PointToPointChannel> m_channel;

  /**
   * The Queue which this PointToPointNetDevice uses as a packet source.
   * Management of this Queue has been delegated to the PointToPointNetDevice
   * and it has the responsibility for deletion.
   * @see class Queue
   * @see class DropTailQueue
   */
  Ptr<Queue> m_queue;

  /**
   * Error model for receive packet events
   */
  Ptr<ErrorModel> m_receiveErrorModel;

  /**
   * The trace source fired when packets come into the "top" of the device
   * at the L3/L2 transition, before being queued for transmission.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxTrace;

  /**
   * The trace source fired when packets coming into the "top" of the device
   * at the L3/L2 transition are dropped before being queued for transmission.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxDropTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * immediately before being forwarded up to higher layers (at the L2/L3
   * transition).  This is a promiscuous trace (which doesn't mean a lot here
   * in the point-to-point device).
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macPromiscRxTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * immediately before being forwarded up to higher layers (at the L2/L3
   * transition).  This is a non-promiscuous trace (which doesn't mean a lot
   * here in the point-to-point device).
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * but are dropped before being forwarded up to higher layers (at the L2/L3
   * transition).
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxDropTrace;

  /**
   * The trace source fired when a packet begins the transmission process on
   * the medium.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyTxBeginTrace;

  /**
   * The trace source fired when a packet ends the transmission process on
   * the medium.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyTxEndTrace;

  /**
   * The trace source fired when the phy layer drops a packet before it tries
   * to transmit it.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyTxDropTrace;

  /**
   * The trace source fired when a packet begins the reception process from
   * the medium -- when the simulated first bit(s) arrive.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyRxBeginTrace;

  /**
   * The trace source fired when a packet ends the reception process from
   * the medium.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyRxEndTrace;

  /**
   * The trace source fired when the phy layer drops a packet it has received.
   * This happens if the receiver is not enabled or the error model is active
   * and indicates that the packet is corrupt.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_phyRxDropTrace;

  /**
   * A trace source that emulates a non-promiscuous protocol sniffer connected
   * to the device.  Unlike your average everyday sniffer, this trace source
   * will not fire on PACKET_OTHERHOST events.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_snifferTrace;

  /**
   * A trace source that emulates a promiscuous mode protocol sniffer connected
   * to the device.  This trace source fire on packets destined for any host
   * just like your average everyday packet sniffer.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_promiscSnifferTrace;

  Ptr<Node> m_node;
  Mac48Address m_address;
  NetDevice::ReceiveCallback m_rxCallback;
  NetDevice::PromiscReceiveCallback m_promiscCallback;
  uint32_t m_ifIndex;
  bool m_linkUp;
  TracedCallback<> m_linkChangeCallbacks;

  static const uint16_t DEFAULT_MTU = 1500;

  /**
   * The Maximum Transmission Unit.  This corresponds to the maximum
   * number of bytes that can be transmitted as seen from higher layers.
   * This corresponds to the 1500 byte MTU size often seen on IP over
   * Ethernet.
   */
  uint32_t m_mtu;

  Ptr<Packet> m_currentPkt;

protected:

  void DoMpiReceive (Ptr<Packet> p);

};

} // namespace ns3

#endif /* POINT_TO_POINT_NET_DEVICE_H */
