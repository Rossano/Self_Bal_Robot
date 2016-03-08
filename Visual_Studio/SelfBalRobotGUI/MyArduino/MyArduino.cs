using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO;
using System.IO.Ports;

namespace Arduino
{
    public class MyArduino
    {
        #region Constants

        public const string get_val_Id = "MPU:";
        public const string cont_get_Id = "cont";
        public const string cont_state_Id = "state";
        public const string lock_Id = "DMP";
        
        #endregion

        #region Members

        protected SerialPort _com;
        private int _baudrate { get; set; }
        public bool connectionStatus { get; private set; }
        protected string readBuffer = "";
        public Queue<string> bufferFIFO = new Queue<string>();
        private bool _continue = false;
        private System.Threading.Thread readThread;

        #endregion

        #region Event Handlers

        private void ArduinoDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            readBuffer = _com.ReadExisting();
            StringReader rd = new StringReader(readBuffer);
            bool endString = false;
            while (!endString)
            {
                try
                {
                    string foo = rd.ReadLine();
                    if (foo.StartsWith(get_val_Id) || foo.StartsWith(cont_get_Id) || foo.StartsWith(cont_state_Id)) bufferFIFO.Enqueue(foo);
                }
                catch (Exception)
                {
                    endString = true;
                }
            }
            //    bufferFIFO.Enqueue(readBuffer);
            //Console.WriteLine("<DBG>: " + readBuffer);
        }
        
        #endregion

        #region Constructor       
 
        public MyArduino(string name, int baud)
        {
            connectionStatus = false;
            _com = new SerialPort(name, baud);
            if(_com == null)
            {
                throw new IOException("Problem creating SerialPort object");
            }

            try
            {
                //_com.DataReceived += new SerialDataReceivedEventHandler(ArduinoDataReceived);
                Thread readThread = new Thread(ReadProc);

                _com.NewLine = Environment.NewLine;
                _com.BaudRate = 9600;
                _com.Parity = Parity.None;
                _com.StopBits = StopBits.One;
                _com.DataBits = 8;
                _com.Handshake = Handshake.None;
                _com.WriteTimeout = 500;
                _com.ReadTimeout = 500;
                // Mandatory for Arduino USB-CDC
                _com.DtrEnable = true;
                _com.RtsEnable = true;
                _com.Open();
                _continue = true;
                readThread.Start();
                connectionStatus = true;
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        #endregion
        
        #region Methods

        public void Disconnect()
        {
            try
            {
                _continue = false;
                readThread.Join();
                _com.Close();
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        public void ReadProc()
        {
            while (_continue)
            {
                try
                {
                    string msg = _com.ReadLine();
                    bufferFIFO.Enqueue(msg);
                }
                catch (TimeoutException) { }
                catch (Exception ex)
                {
                    throw ex;
                }
            }
        }

        public string getBuffer()
        {
            if (bufferFIFO.Count != 0) readBuffer = bufferFIFO.Dequeue();
            else return string.Empty;
            if (readBuffer.Length != 0)
            {
                string foo = readBuffer;                
                readBuffer = "";
                return foo;
            }
            else return "";
        }

        public Queue<string> getFIFOBuffer()
        {
            return bufferFIFO;
        }

        public void arduinoWrite(string str)
        {
            if (_com.IsOpen)
            {
                try
                {
                    _com.WriteLine(str);
                }
                catch (Exception ex)
                {
                    throw ex;
                }
            }
        }

        public void setArduinoPort(int baud, System.IO.Ports.StopBits _stop, System.IO.Ports.Parity _parity, System.IO.Ports.Handshake _hand)
        {
            try
            {
                _com.Close();
                _com.BaudRate = baud;
                _com.StopBits = _stop;
                _com.Parity = _parity;
                _com.Handshake = _hand;
                _com.Open();
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public SerialPort getArduinoPort()
        {
            return _com;
        }

        #endregion

    }
}
