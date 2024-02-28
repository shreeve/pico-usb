## Frequently Asked Questions or Concepts

### 1. Single vs Double Buffering

**Question:** On the rp2040 microcontroller, the USB Controller allows for
interrupt generation for either of two options. First, it can generate an
interrupt every time a transfer buffer is full. Second, it can generate an
interrup every time both transfer buffers are full. If I enable single
buffering, then we only have one buffer to deal with anyway, so the second case
is irrelevant. However, if I enable double buffering, then this question becomes
relevant. My question is this: If I enable double buffering, but then request to
be interrupted every time a single buffer is full, do I lose the advantage of
double buffering in the first place?

---

**Answer:** The RP2040 microcontroller's USB Controller's ability to generate
interrupts when transfer buffers are full, and the choice between single
buffering and double buffering, is an important aspect of managing USB data
transfers efficiently. Let's explore the implications of your setup.

#### Double Buffering Explained
Double buffering involves using two buffers to manage data transfers between the
USB controller and the microcontroller's processing core. While one buffer is
being filled with incoming data from USB transactions, the other buffer can be
processed by the microcontroller, and vice versa. This approach aims to minimize
idle time for both the USB interface and the processor, allowing for more
efficient data handling, especially in high-speed or continuous data transfer
scenarios.

#### Interrupt Generation Options

* Interrupt on Every Buffer Full: Generating an interrupt every time a buffer
  becomes full allows the microcontroller to process data more frequently. This
  setup ensures that data is handled promptly as it arrives, which can be
  beneficial for time-sensitive data processing.

* Interrupt on Both Buffers Full: Alternatively, generating an interrupt only
  when both buffers are full might reduce the interrupt load on the
  microcontroller. This approach can be more efficient in scenarios where
  processing overhead is a concern, and there's confidence that the
  microcontroller can process both buffers' worth of data before new data
  overflows.

#### Impact on Double Buffering Efficiency
With Single Buffering: The choice of interrupt generation strategy is
straightforward since there's only one buffer. An interrupt is generated
whenever this single buffer is full.

With Double Buffering and Interrupt on Single Buffer Full:

**Advantages Retained:** Even if you choose to generate an interrupt every time
a single buffer is full, you still retain the core advantage of double
buffering. While one buffer is being processed in response to the interrupt, the
other buffer can continue to receive data from subsequent USB transactions. This
setup maintains the throughput advantage of double buffering by avoiding the
need for USB data transfers to wait while the microcontroller is processing
data.

**Considerations:** The key here is ensuring that the microcontroller processes
each buffer's data quickly enough to clear the buffer before the next buffer
fills up and requires processing. If the microcontroller can keep up with this
pace, you don't lose the advantages of double buffering; you maintain continuous
data flow with minimal idle time.

**Summary:** Enabling double buffering and configuring interrupts to trigger
every time a single buffer is full does not inherently negate the advantages of
double buffering. The effectiveness depends on the processing capability and
speed of the microcontroller relative to the rate of incoming data. If the
microcontroller can efficiently process data from one buffer while the other is
being filled, double buffering remains advantageous, offering continuous data
handling with reduced risk of data loss or delays.

### 2. Clock Timing and Register Access

When clk_sys (usually 133Mhz) and clk_usb (usually 48MHz) are not the same, the
processor and the USB controller run at different speeds. To properly coordinate
them, we must sometimes waste clk_sys cycles to allow time for clk_usb to catch
up. Each clk_sys cycle is 133/48 times faster than a clk_usb cycle, which is
2.77 (roughly 3) times as fast. So, for each 1 clk_usb cycle, we should waste 3
clk_sys cycles.

For the USB controller, the START_TRANS bit in SCR and the AVAILABLE bit in BCR
need special care. These bits trigger processor actions when they are set, but
they will execute too soon since the USB controller needs more time to perform
the actions specified in the other bits. Thus, we need to set the USB specific
bits first, delay a few cycles, and then set the bits for the processor. The
datasheet shows how long to wait:

For SCR, Datasheet § 4.1.2.7 (p. 390) says START_TRANS needs two clk_usb For
BCR, Datasheet § 4.1.2.5.1 (p. 383) says AVAILABLE needs one clk_usb

We have several values to set, so we order them as shown below. Notice that this
sets SCR without START_TRANS and then, in 6 cycles, it sets it again but this
time including START_TRANS. BCR is similar, but will be set again after 3
cycles. The setting of DAR and two NOP's are inserted to make everything line up
correctly.
