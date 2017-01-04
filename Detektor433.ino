/*
  Sketch zur Vorab-Analyse unbekannter 433-MHZ-Wettersensoren
  und Fernbedienungen von 433MHz-Funksteckdosen
  http://forum.iobroker.net/viewtopic.php?f=35&t=3831&p=35623#p35623
  Inspiriert durch Beiträge im Arduino-Forum:
  http://arduino.cc/forum/index.php/topic,119739.0.html
  http://arduino.cc/forum/index.php/topic,136836.0.html
  
  Hardware:
  1. Arduino-Board mit 433 MHz Regenerativempfänger für ASK/OOK,
  angeschlossen an einem interruptfähigen Pin.
  2. Funksensor entweder eines 433 MHz Funkthermometers
  oder Funk-Wetterstation oder Steckdosen-Funkfernbedienung
  
  Analysiert werden können Sensoren mit folgendem Funkprotokoll:
  - extra langes Startbit (extra langer LOW Pulse am Receiver)
  - langes 1-Datenbit (langer LOW Pulse am Receiver)
  - kurzes 0-Datenbit (kurzer LOW Pulse am Receiver)
  - sehr kurze Trenn-Impulse zwischen den Datenbits (sehr kurze HIGH-Impulse am Receiver)
  - 20 bis 50 Datenbits pro Datenpaket
  Diese Art Funkprotokoll trifft auf die meisten billigen 433 MHZ
  Funkthermometer, Funk-Wetterstationen und Funk-Steckdosen zu.
  
  Ausgabe ueber den seriellen Monitor
  Je erkanntem Datenpaket am Receiver wird ausgegeben:
  - Länge des Startbits (Mikrosekunden LOW) und des nachfolgenden HIGH-Impulses
  - Anzahl der erkannten Datenbits im Datenpaket
  - Länge aller erkannten Datenbits (Mikrosekunden LOW)
  - Länge der zwischen den Datenbits erkannten Pausen (Mikrosekunden HIGH)
  - die als 0/1-Bitstrom decodierten Datenbits des Datenpakets
  
  Nur Vorab-Analyse des Timings im Funkprotokoll!
  In einem weiteren Schritt muss dann die Bedeutung der Bits
  und die Umsetzung in Messwerte erst noch detalliert decodiert werden,
  dieser Sketch erkennt nur das Timing und die Groesse der Datenpakete!
*/

// connect data pin of rx433 module to a pin that can handle hardware interrupts
// with an Arduino UNO this is digital I/O pin 2 or 3 only
#define RX433DATAPIN 2

// hardware interrupt connected to the pin
// with Arduino UNO interrupt-0 belongs to pin-2, interrupt-1 to pin-3
#define RX433INTERRUPT 0

// Set speed of serial in Arduino IDE to the following value
#define SERIALSPEED 115200

// Now make some suggestions about pulse lengths that may be detected
// minimum duration (microseconds) of the start pulse
#define MINSTARTPULSE 2000///740

// minimum duration (microseconds) of a short bit pulse
#define MINBITPULSE 800 //250

// minimum duration (microseconds) of a HIGH pulse between valid bits
#define MINHITIME 640 //50

// variance between pulses that should have the same duration
#define PULSEVARIANCE 250

// minimum count of data bit pulses following the start pulse
#define MINPULSECOUNT 40

// maximum count of data bit pulses following the start pulse
#define MAXPULSECOUNT 70

// buffer sizes for buffering pulses in the interrupt handler
#define PBSIZE 216

// monitor pin 
#define MONITOR_PIN 12

volatile unsigned int pulsbuf[PBSIZE]; // ring buffer storing LOW pulse lengths
volatile unsigned int hibuf[PBSIZE]; // ring buffer storing HIGH pulse lengths
unsigned int validpulsbuf[MAXPULSECOUNT]; // linear buffer storing valid LOW pulses
unsigned int validhibuf[MAXPULSECOUNT]; // linear buffer storing valid HIGH pulses

volatile byte pbread,pbwrite; // read and write index into ring buffer

boolean counting;
byte i,counter;
int startBitDurationL,startBitDurationH,shortBitDuration,longBitDuration;

//--- prototypes
void rx433Handler(); 
//----------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Start!");
  pinMode(MONITOR_PIN, OUTPUT);
  pinMode(RX433DATAPIN, INPUT);
  attachInterrupt(RX433INTERRUPT, rx433Handler, CHANGE);
}
//----------------------------------------
void rx433Handler()
{
  static long rx433LineUp, rx433LineDown;
  long LowVal, HighVal;
  int rx433State = digitalRead(RX433DATAPIN); // current pin state
  if (rx433State) // pin is now HIGH
  {
    digitalWrite(MONITOR_PIN, HIGH); 
    rx433LineUp=micros(); // line went HIGH after being LOW at this time
    LowVal=rx433LineUp - rx433LineDown; // calculate the LOW pulse time
    if (LowVal>MINBITPULSE)
    { // store pulse in ring buffer only if duration is longer than MINBITPULSE
      // To be able to store startpulses of more than Maxint duration, we dont't store the actual time,
      // but we store MINSTARTPULSE+LowVal/10, be sure to calculate back showing the startpulse length!
      if (LowVal>MINSTARTPULSE) LowVal=MINSTARTPULSE+LowVal/10; // we will store this as unsigned int, so do range checking
  
      pulsbuf[pbwrite]=LowVal; // store the LOW pulse length
      pbwrite++; // advance write pointer in ringbuffer
      if (pbwrite>=PBSIZE) pbwrite=0; // ring buffer is at its end
    }
    digitalWrite(MONITOR_PIN, LOW); 
  }
  else
  {
    digitalWrite(MONITOR_PIN, HIGH); 
    rx433LineDown=micros(); // line went LOW after being HIGH
    HighVal = rx433LineDown - rx433LineUp; // calculate the HIGH pulse time
    if (HighVal>31999) HighVal=31999; // we will store this as unsigned int
    hibuf[pbwrite] = HighVal; // store the HIGH pulse length
    digitalWrite(MONITOR_PIN, LOW); 
  }
}
//----------------------------------------
void showBuffer()
// this function will show the results on the serial monitor
// output will be shown if more bits than MINPULSECOUNT have been collected
{
  long sum;
  int avg;
  sum=0;
  if (counter>=MINPULSECOUNT)
  { // only show buffer contents if it has enough bits in it
    Serial.println();
    Serial.print("Start Bit L: "); Serial.print((startBitDurationL-MINSTARTPULSE)*10L);
    Serial.print(" H: ");Serial.println(startBitDurationH);
    Serial.print("Data Bits: ");Serial.println(counter);
    Serial.print("L: ");
    for (i=0;i<counter;i++)
    {
      Serial.print(validpulsbuf[i]);Serial.print(" ");
      sum+=validpulsbuf[i];
    }
    Serial.println();

    Serial.print("H: ");
    for (i=0;i<counter;i++)
    {
      Serial.print(validhibuf[i]);Serial.print(" ");
    }
    Serial.println();

    avg=sum/counter; // calculate the average pulse length
    // then assume that 0-bits are shorter than avg, 1-bits are longer than avg
    for (i=0;i<counter;i++)
    {
      if (validpulsbuf[i]<avg) Serial.print('0'); else Serial.print('1');
    }
    Serial.println();

  }
  counting=false;
  counter=0;
}
//----------------------------------------
void loop()
{
  long lowtime, hitime;
  if (pbread!=pbwrite) // check for data in ring buffer
  {
    lowtime=pulsbuf[pbread];    // read data from ring buffer
    hitime=hibuf[pbread];
    cli();                      // Interrupts off while changing the read pointer for the ringbuffer
    pbread++;
    if (pbread>=PBSIZE) pbread=0;
    sei();                      // Interrupts on again
    if (lowtime>MINSTARTPULSE)  // we found a valid startbit!
    {
      if (counting) showBuffer();   // new buffer starts while old is still counting, show it first
      startBitDurationL=lowtime;
      startBitDurationH=hitime;
      counting=true;    // then start collecting bits
      counter=0;        // no data bits yet
    }
    else if (counting && (counter==0)) // we now see the first data bit
    { // this may be a 0-bit or a 1-bit, so make som  e assumption about max/min lengths of data bits that will follow
      shortBitDuration=lowtime/2;
      if (shortBitDuration<MINBITPULSE+PULSEVARIANCE)
        shortBitDuration=MINBITPULSE;
      else
        shortBitDuration-=PULSEVARIANCE;
      longBitDuration=lowtime*2+PULSEVARIANCE;
      validpulsbuf[counter]=lowtime;
      validhibuf[counter]=hitime;
      counter++;
    }
    else if (counting&&(lowtime>shortBitDuration)&&(lowtime<longBitDuration))
    {
      validpulsbuf[counter]=lowtime;
      validhibuf[counter]=hitime;
      counter++;
      if ((counter==MAXPULSECOUNT) || (hitime<MINHITIME))
      {
        showBuffer();
      }
    }
    else // Low Pulse is too short
    {
      if (counting) showBuffer();
      counting=false;
      counter=0;
    }
  }
}
//----------------------------------------
// <EOF>
