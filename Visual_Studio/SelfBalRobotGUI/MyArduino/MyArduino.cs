using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.IO.Ports;

namespace Arduino
{
    public class MyArduino
    {
        #region Members
        protected SerialPort _com;
        private int _baudrate { get; set; }
        public bool connectionStatus { get; private set; }
        protected string readBuffer = "";
        #endregion

        #region Event Handlers
        private void ArduinoDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            readBuffer = _com.ReadExisting();
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
                _com.DataReceived += new SerialDataReceivedEventHandler(ArduinoDataReceived);
                _com.BaudRate = 57600;
                _com.Parity = Parity.None;
                _com.StopBits = StopBits.One;
                _com.DataBits = 8;
                _com.Handshake = Handshake.None;
                _com.WriteTimeout = 10000;
                _com.Open();
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
                _com.Close();
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        public string getBuffer()
        {
            if (readBuffer.Length != 0)
            {
                string foo = readBuffer;
                readBuffer = "";
                return foo;
            }
            else return "";
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
