

use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}, spi::SpiBus};

#[repr(u8)]
#[derive(ufmt::derive::uDebug)]
pub enum Errors{
    WriteByte,
    WriteBytes,
    ReadByte,
    ReadBytes,
    ReadBytesBufTooSmall,
    InputPinError
}

type Result<T> = core::result::Result<T, Errors>;


pub struct SX127x<Delay, SPI, CsPin, ResetPin, RxTxDonePin>{
    delay : Delay,
    spi : SPI,
    cs_pin : CsPin,
    reset_pin : ResetPin,
    rx_tx_done_pin : RxTxDonePin
}


impl<Delay, SPI, CsPin, ResetPin, RxTxDonePin> SX127x<Delay, SPI, CsPin, ResetPin, RxTxDonePin> 
where 
    Delay : DelayNs,
    SPI : SpiBus,
    CsPin : OutputPin,
    ResetPin : OutputPin,
    RxTxDonePin : InputPin
{
    pub fn new( delay: Delay, spi : SPI, cs_pin : CsPin, reset_pin : ResetPin, rx_tx_done_pin : RxTxDonePin) -> Self {
        SX127x{
            delay,
            spi,
            cs_pin,
            reset_pin,
            rx_tx_done_pin,
        }
    }

    pub fn init(&mut self) -> Result<()>{
        //         // adapted from https://github.com/xg590/Learn_SX1276.git
        // // SX1276_Lora.py
        // // I copied outcommented python chunks to document what happens here.

        // Reset
        {
            self.reset_pin.set_low();
            self.delay.delay_ms(10);
            self.reset_pin.set_high();
            self.delay.delay_ms(10);
        }

        // let r = spi_device.read_byte(Registers::RegOpMode).await?;
        // info!("RegOpMode before switch = {}",r);

        // // Choose LoRa (instead of FSK) mode for SX1276 and put the module in sleep mode
        self.write_byte(Registers::RegOpMode, Mode::SLEEP as u8)?;
        self.delay.delay_ms(10);
        self.write_byte(Registers::RegOpMode, Mode::SLEEP as u8 | 1 << 7)?;

        // // # Test read function
        // // r = self.read('RegOpMode')
        // // print( r )
        // // assert r == (self.Mode['SLEEP'] | LongRangeMode << 7), "LoRa initialization failed"
        // let r = spi_device.read_byte(Registers::RegOpMode).await?;
        // info!("RegOpMode after switch = {}",r);

        // # Set modem config: bandwidth, coding rate, header mode, spreading factor, CRC, and etc.
        // # See 4.4. LoRa Mode Register Map
        // Bw                   = {'125KHz':0b0111, '500kHz':0b1001}
        // CodingRate           = {5:0b001, 6:0b010, 7:0b011, 8:0b100}
        // ImplicitHeaderModeOn = {'Implicit':0b1, 'Explicit':0b0}
        // self.write('RegModemConfig1', Bw['125KHz'] << 4 | CodingRate[5] << 1 | ImplicitHeaderModeOn['Explicit'])
        self.write_byte(Registers::RegModemConfig1, BandWidth::_125kHz as u8 | CodingRate::_5 as u8 | ImplicitHeaderModeOn::Explicit as u8)?;
        // SpreadingFactor  = {7:0x7, 9:0x9, 12:0xC}
        // TxContinuousMode = {'normal':0b0, 'continuous':0b1}
        // RxPayloadCrcOn   = {'disable':0b0, 'enable':0b1}
        // self.write('RegModemConfig2', SpreadingFactor[7] << 4 | TxContinuousMode['normal'] << 3 | RxPayloadCrcOn['enable'] << 2 | 0x00)
        self.write_byte(Registers::RegModemConfig2, SpreadingFactor::_7 as u8 | TxContinuousMode::Normal as u8 | RxPayloadCrcOn::Enable as u8)?;
        // LowDataRateOptimize = {'Disabled':0b0, 'Enabled':0b1}
        // AgcAutoOn = {'register LnaGain':0b0, 'internal AGC loop':0b1}
        // self.write('RegModemConfig3', LowDataRateOptimize['Enabled'] << 3 | AgcAutoOn['internal AGC loop'] << 2)
        self.write_byte(Registers::RegModemConfig3, LowDataRateOptimize::Disabled as u8 | AgcAutoOn::InternalAGCLoop as u8)?;

        // # Preamble length
        // self.write('RegPreambleMsb', 0x0) # Preamble can be (2^15)kb long, much longer than payload
        self.write_byte(Registers::RegPreambleMsb, 0)?;
        // self.write('RegPreambleLsb', 0x8) # but we just use 8-byte preamble
        self.write_byte(Registers::RegPreambleLsb, 8)?;

        // # See 4.1.4. Frequency Settings
        // FXOSC = 32e6 # Freq of XOSC
        // FSTEP = FXOSC / (2**19)
        // Frf = int(868000000 / FSTEP)
        // self.write('RegFrfMsb', (Frf >> 16) & 0xff)
        // self.write('RegFrfMid', (Frf >>  8) & 0xff)
        // self.write('RegFrfLsb',  Frf        & 0xff)
        const FX_OSC : f64 = 32e6;
        const F_STEP : f64 = FX_OSC / ((1<<19) as f64);
        const FRF : u32 = (868_000_000.0 / F_STEP) as u32;
        self.write_byte(Registers::RegFrfMsb, ((FRF >> 16) & 0xff) as u8 )?;
        self.write_byte(Registers::RegFrfMid, ((FRF >> 08) & 0xff) as u8 )?;
        self.write_byte(Registers::RegFrfLsb, ((FRF >> 00) & 0xff) as u8 )?;

        // # Output Power
        // '''
        // If desired output power is within -4 ~ +15dBm, use PA_LF or PA_HF as amplifier.
        // Use PA_BOOST as amplifier to output +2 ~ +17dBm continuous power or up to 20dBm
        //   peak power in a duty cycled operation.
        // Here we will always use PA_BOOST.
        // Since we use PA_BOOST, Pout = 2 + OutputPower and MaxPower could be any number (Why not 0b111/0x7?)
        // '''
        // PaSelect    = {'PA_BOOST':0b1, 'RFO':0b0} # Choose PA_BOOST (instead of RFO) as the power amplifier
        // MaxPower    = {'15dBm':0x7, '13dBm':0x2}  # Pmax = 10.8 + 0.6 * 7
        // OutputPower = {'17dBm':0xf, '2dBm':0x0}
        // self.write('RegPaConfig', PaSelect['PA_BOOST'] << 7 | MaxPower['15dBm'] << 4 | OutputPower['2dBm'])
        self.write_byte(Registers::RegPaConfig, PaSelect::PaBoost as u8 | MaxPower::_15dBm as u8 | OutputPower::_2dBm as u8)?;

        // # Enables the +20dBm option on PA_BOOST pin.
        // if plus20dBm: # PA (Power Amplifier) DAC (Digital Analog Converter)
        //     PaDac = {'default':0x04, 'enable_PA_BOOST':0x07} # Can be 0x04 or 0x07. 0x07 will enables the +20dBm option on PA_BOOST pin
        //     self.write('RegPaDac', PaDac['enable_PA_BOOST'])
        const PLUS20_DBM : bool = false;
        if PLUS20_DBM {
            self.write_byte(Registers::RegPaDac, PaDac::EnablePaBoost as u8)?;
        }

        // # FIFO data buffer 
        // '''
        // SX1276 has a 256 byte memory area as the FIFO buffer for Tx/Rx operations.
        // How do we know which area is for Tx and which is for Rx.
        // We must set the base addresses RegFifoTxBaseAddr and RegFifoRxBaseAddr independently.
        // Since SX1276 work in a half-duplex manner, we better set both base addresses
        // at the bottom (0x00) of the FIFO buffer so that we can buffer 256 byte data
        // during transmition or reception.
        // ''' 
        // self.Fifo_Bottom = 0x00 # We choose this value to max buffer we can write (then send out)
        // self.write('RegFifoTxBaseAddr', self.Fifo_Bottom)
        // self.write('RegFifoRxBaseAddr', self.Fifo_Bottom)
        const FIFO_BOTTOM : u8 = 0x0;
        self.write_byte(Registers::RegFifoTxBaseAddr, FIFO_BOTTOM)?;
        self.write_byte(Registers::RegFifoRxBaseAddr, FIFO_BOTTOM)?;

        // let sender = CHANNEL.sender();

        // // launch the task which shall handle interrupts
        // _spawner.must_spawn(handle_sx127x_dio0_interrupt(self.interrupt_input_mutex, self.spi_device_mutex, sender));

        Ok(())

    }


    /// set into continuous receive mode.
    #[allow(dead_code)]
    pub fn start_rx_cont(&mut self) -> Result<()>{

        // self.write('RegDioMapping1', self.DioMapping['Dio0']['RxDone'])  
        self.write_byte(Registers::RegDioMapping1, Dio0Mapping::RxDone as u8)?;

        // self.write('RegOpMode', self.Mode['RXCONTINUOUS']) # Request Standby mode so SX1276 send out payload   
        self.write_byte(Registers::RegOpMode, Mode::RXCONTINUOUS as u8)?;

        // clear interrupt flags, just in case.s
        self.write_byte(Registers::RegIrqFlags, 0xff)?;

        Ok(())
    }

    /// poll rx_tx_done_pin until data is received, write data
    /// into buf, return number of received bytes.
    pub fn wait_and_receive(&mut self, buf : &mut [u8]) -> Result<u8>{

        // poll for change to high on RxTxDonePin
        while self.rx_tx_done_pin.is_low().or(Err(Errors::InputPinError))? {}

        // Get number of received bytes and retrieve them into buf, check buf size.
        let result = self.read_byte(Registers::RegRxNbBytes).and_then( |rx_bytes: u8|{
                if (rx_bytes as usize) < buf.len(){
                    self.read_bytes(Registers::RegFifo, &mut buf[0..rx_bytes as usize])?;
                    Ok(rx_bytes)
                }
                else {
                    Err(Errors::ReadBytesBufTooSmall)
                }
            }
        );

        // clear interrupt flag in sx127x 
        // (i.e. make RxTxDonePin go low)
        // clear interrupt flags
        self.write_byte(Registers::RegIrqFlags, 0xff)?;

        result
    }

    #[deprecated="not implemented"]
    pub fn tx_data(&mut self, buf: &[u8]) -> Result<()>{
        let opmode = self.read_byte(Registers::RegOpMode)?;
        Err(Errors::WriteByte)
    }
    

    // Very special for the Atmega328 at 1Mhz.
    // 2us when using 1Mhz. This should be enough.
    #[inline(always)]
    fn cs_delay(){
        unsafe {
            core::arch::asm!("nop", "nop");
        }
    }

    // def write(self, reg, data):
    //     wb = bytes([self.RegTable[reg] | 0x80]) # Create a writing byte
    //     if isinstance(data, int):
    //         data = wb + bytes([data])
    //     elif isinstance(data, str):
    //         data = wb + bytes(data, 'utf-8')
    //     elif isinstance(data, bytes):
    //         data = wb + data
    //     else:
    //         raise Exception('unknown type of data: ' + str(type(data)))
    //     self.cs_pin.value(0) # Bring the CS pin low to enable communication
    //     self.spi.write(data)
    //     self.cs_pin.value(1) # release the bus.
    
    /// write a single byte. Calls write_bytes.
    #[allow(dead_code)]
     pub fn write_byte(&mut self, reg : Registers, value: u8) -> Result<()>{
        self.write_bytes(reg, &[value])
    }

    /// write bytes to register address. 
    #[allow(dead_code)]
    pub fn write_bytes(&mut self, reg : Registers, bytes: &[u8]) -> Result<()>{
        self.cs_pin.set_low();
        Self::cs_delay();
        let result = 
            self.spi.write(&[(reg as u8) | 0x80 ]).and_then(
                |_| self.spi.write(bytes));
        self.cs_pin.set_high();
        Self::cs_delay();
        result.or(Err(Errors::WriteBytes))
    }

    // def read(self, reg=None, length=1):
    //     self.cs_pin.value(0)
    //     # https://docs.micropython.org/en/latest/library/machine.SPI.html#machine-softspi
    //     if length == 1:
    //         data = self.spi.read(length+1, self.RegTable[reg])[1]
    //     else:
    //         data = self.spi.read(length+1, self.RegTable[reg])[1:]
    //     self.cs_pin.value(1)
    //     return data

    /// read a single byte. Uses read_bytes.
    #[allow(dead_code)]
    pub fn read_byte(&mut self, reg: Registers) -> Result<u8> {
        let mut buf = [0u8;1];
        self.read_bytes(reg, &mut buf).and(Ok(buf[0]))

    }

     /// read bytes into provided buf.
     #[allow(dead_code)]
     pub fn read_bytes(&mut self, reg: Registers, buf : &mut [u8]) -> Result<()>{

        self.cs_pin.set_low();
        Self::cs_delay();
        let result = 
            self.spi.write(&[reg as u8]).and_then(
                |()|  self.spi.read(buf)
        );
        self.cs_pin.set_high();
        Self::cs_delay();
        result.or(Err(Errors::ReadBytes))
     }

}


    

    #[allow(dead_code)]
#[repr(u8)]
pub enum Registers{
    RegFifo              = 0x00 ,
    RegOpMode            = 0x01 , // operation mode
    RegFrfMsb            = 0x06 ,
    RegFrfMid            = 0x07 ,
    RegFrfLsb            = 0x08 ,
    RegPaConfig          = 0x09 ,
    RegFifoTxBaseAddr    = 0x0e ,
    RegFifoRxBaseAddr    = 0x0f ,
    RegFifoAddrPtr       = 0x0d ,
    RegFifoRxCurrentAddr = 0x10 ,
    RegIrqFlags          = 0x12 ,
    RegRxNbBytes         = 0x13 , // Number of received bytes
    RegPktSnrValue       = 0x19 ,
    RegPktRssiValue      = 0x1a ,
    RegRssiValue         = 0x1b ,
    RegModemConfig1      = 0x1d ,
    RegModemConfig2      = 0x1e ,
    RegPreambleMsb       = 0x20 ,
    RegPreambleLsb       = 0x21 ,
    RegPayloadLength     = 0x22 ,
    RegModemConfig3      = 0x26 ,
    RegDioMapping1       = 0x40 ,
    RegVersion           = 0x42 ,
    RegPaDac             = 0x4d
}



/// see Table 16 LoRa ® Operating Mode Functionality
#[allow(non_snake_case)]
mod Mode{
    #![allow(dead_code)]
    pub const SHIFT             : u8 = 0;
    pub const MASK              : u8 = 0b111 << SHIFT;

    pub const SLEEP             : u8 = 0b000 << SHIFT;
    pub const STANDBY           : u8 = 0b001 << SHIFT;
    pub const TX                : u8 = 0b011 << SHIFT;
    pub const RXCONTINUOUS      : u8 = 0b101 << SHIFT;
    pub const RXSINGLE          : u8 = 0b110 << SHIFT;
    pub const CAD               : u8 = 0b111 << SHIFT;
}


/// Bw                   = {'125KHz':0b0111, '500kHz':0b1001}
#[allow(non_snake_case)]
mod BandWidth{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]
    pub const SHIFT   : u8 = 4;
    pub const MASK    : u8 = 0b1111 << SHIFT;

    pub const _125kHz : u8 = 0b0111 << SHIFT;
    pub const _500kHz : u8 = 0b1001 << SHIFT;

}


/// CodingRate           = {5:0b001, 6:0b010, 7:0b011, 8:0b100}
#[allow(non_snake_case)]
pub mod CodingRate {
    #![allow(dead_code)]
    pub const SHIFT   : u8 = 1;
    pub const MASK    : u8 = 0b111 << SHIFT;

    pub const _5 : u8 = 0b001 << SHIFT;
    pub const _6 : u8 = 0b010 << SHIFT;
    pub const _7 : u8 = 0b011 << SHIFT;
    pub const _8 : u8 = 0b100 << SHIFT;
}

#[allow(non_snake_case)]
pub mod ImplicitHeaderModeOn{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]
 
    pub const SHIFT   : u8 = 0;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const Implicit : u8 = 0b1;
    pub const Explicit : u8 = 0b0;

}



#[allow(non_snake_case)]
pub mod SpreadingFactor{
    #![allow(dead_code)]
    pub const SHIFT   : u8 = 4;
    pub const MASK    : u8 = 0b1111 << SHIFT;

    pub const _6  : u8 = 0x6 << SHIFT;
    pub const _7  : u8 = 0x7 << SHIFT;
    pub const _8  : u8 = 0x8 << SHIFT;
    pub const _9  : u8 = 0x9 << SHIFT;
    pub const _10 : u8 = 0xA << SHIFT;
    pub const _11 : u8 = 0xB << SHIFT;
    pub const _12 : u8 = 0xC << SHIFT;

}


#[allow(non_snake_case)]
pub mod TxContinuousMode{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 3;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const Normal     : u8 = 0b0 << SHIFT;
    pub const Continuous : u8 = 0b1 << SHIFT;
}



#[allow(non_snake_case)]
pub mod RxPayloadCrcOn{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 2;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const Disable : u8 = 0b0 << SHIFT;
    pub const Enable  : u8 = 0b1 << SHIFT;
}



#[allow(non_snake_case)]
pub mod LowDataRateOptimize{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 3;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const Disabled : u8 = 0b0 << SHIFT;
    pub const Enabled  : u8 = 0b1 << SHIFT;
}



#[allow(non_snake_case)]
pub mod AgcAutoOn{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 2;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const RegisterLnaGain : u8 = 0b0 << SHIFT;
    pub const InternalAGCLoop  : u8 = 0b1 << SHIFT;
}



/// PaSelect    = {'PA_BOOST':0b1, 'RFO':0b0} # Choose PA_BOOST (instead of RFO) as the power amplifier
#[allow(non_snake_case)]
pub mod PaSelect{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 7;
    pub const MASK    : u8 = 0b1 << SHIFT;

    pub const Rfo : u8 = 0b0 << SHIFT;
    pub const PaBoost  : u8 = 0b1 << SHIFT;
}



/// MaxPower    = {'15dBm':0x7, '13dBm':0x2}  # Pmax = 10.8 + 0.6 * 7
#[allow(non_snake_case)]
pub mod MaxPower{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 4;
    pub const MASK    : u8 = 0b111 << SHIFT;

    pub const _15dBm : u8 = 0x7 << 4;
    pub const _13dBm : u8 = 0x2 << 4;

}


/// OutputPower = {'17dBm':0xf, '2dBm':0x0}
#[allow(non_snake_case)]
pub mod OutputPower{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 0;
    pub const MASK    : u8 = 0b1111 << SHIFT;

    pub const _17dBm : u8 = 0xf << SHIFT;
    pub const _2dBm  : u8 = 0x0 << SHIFT;

}



/// PaDac = {'default':0x04, 'enable_PA_BOOST':0x07}
/// Can be 0x04 or 0x07. 0x07 will enables the +20dBm option on PA_BOOST pin
#[allow(non_snake_case)]
pub mod PaDac{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 0;
    pub const MASK    : u8 = 0b111 << SHIFT;

    pub const Default       : u8 = 0x04 << SHIFT;
    pub const EnablePaBoost : u8 = 0x07 << SHIFT;

}



#[allow(non_snake_case)]
pub mod Dio0Mapping{
    #![allow(dead_code)]
    #![allow(non_upper_case_globals)]

    pub const SHIFT   : u8 = 6;
    pub const MASK    : u8 = 0b11 << SHIFT;

    pub const RxDone  : u8 = 0x00 << SHIFT;
    pub const TxDone  : u8 = 0x01 << SHIFT;
    pub const CadDone : u8 = 0x02 << SHIFT;

}