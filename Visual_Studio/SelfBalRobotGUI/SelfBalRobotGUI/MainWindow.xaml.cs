using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Arduino;
using System.Threading;

namespace SelfBalRobotGUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constants

        const string request_data = "get_val";
        const string request_feedback = "get_fb";
        const string request_info = "info";
        const string set_fb_coeff = "set_coeff";
        
        #endregion

        #region Members
        private System.Windows.Threading.DispatcherTimer readTimer;
        private TimeSpan _timerInterval;
        private SelfBalRobotGUI_Core _robot;
        private string readBuf;
        private bool isConnected;
        private string portName = "COM4";
        private int baudRate = 57600;
        #endregion

        #region Constructor

        public MainWindow()
        {
            InitializeComponent();
            // Initialize timer
            readTimer = new System.Windows.Threading.DispatcherTimer();
            readTimer.Tick += readTick;
            _timerInterval = new TimeSpan(0, 0, 0, 0, 500);
            readTimer.Interval = _timerInterval;
            foreach(string s in System.IO.Ports.SerialPort.GetPortNames())
            {
                ComboBoxCOM.Items.Add(s);                
            }
            //readTimer.Start();
            // Initialize variables
            isConnected = false;            
        }
        
        #endregion

        #region Event Handlers

        private void ConnectArduinoButto_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                portName = ComboBoxCOM.SelectedItem.ToString();
                _robot = new SelfBalRobotGUI_Core(portName, baudRate);
                isConnected = true;
                System.Threading.Thread.Sleep(2000);
                string foo = _robot.getBuffer();
                readTimer.Start();
            }
            catch (Exception ex)
            {
                //throw ex;
                errMessage(ex.Message);
            }
        }

        private void readTick(object sender, EventArgs e)
        {
            if(_robot.connectionStatus)
            {
                try
                {
                    requestIMUData();
//                    Thread.Sleep(10);
                    readBuf = _robot.getBuffer();
                    _robot.getControllerState();
                    char[] separators = { ' ' };
                    string[] tokens = readBuf.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                    YawAngle.Content = tokens[1];
                    RollAngle.Content = tokens[2];
                    PitchAngle.Content = tokens[3];
                    GyroSpeed.Content = tokens[4];
                    Theta.Content = _robot.K[0];
                    ThetaDot.Content = _robot.K[1];
                    X.Content = _robot.K[2];
                    Xdot.Content = _robot.K[3];
                }
                catch (Exception ex)
                {
                    throw ex;
                }
            }
            else
            {
                throw new Exception("Try to read on COM not open");
            }
        }

        private void ArduinoCOM_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            portName = sender.ToString();
        }

        private void ControllerEnableCheckBox_Click(object sender, RoutedEventArgs e)
        {
            _robot.controllerToggle((bool)ControllerEnableCheckBox.IsChecked);
        }

        private void ContrSet_Button_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                float k1 = (float)Convert.ToDouble(K1_Text.Text);
                float k2 = (float)Convert.ToDouble(K2_Text.Text);
                float k3 = (float)Convert.ToDouble(K3_Text.Text);
                float k4 = (float)Convert.ToDouble(K4_Text.Text);
                _robot.setControllerFeedback(k1, k2, k3, k4);
            }                             
            catch(Exception ex)
            {
                errMessage(ex.Message);
            }
        }

        #endregion

        #region Methods

        void requestIMUData()
        {
            _robot.arduinoWrite(request_data);
        }

        void changeReadInterval(int mills)
        {
            int sec = mills / 1000;
            mills -= sec * 1000;
            _timerInterval = new TimeSpan(0, 0, sec, mills);
        }

        void requestFeedbackData()
        {
            _robot.arduinoWrite(request_feedback);
        }

        void requestInfo()
        {
            _robot.arduinoWrite(request_info);
        }

        void setFeedbackCoeff(double k1, double k2, double k3, double k4)
        {
            string cmd = set_fb_coeff + " " + k1.ToString() + " " + k2.ToString() + " " + k3.ToString() + " " + k4.ToString();
            _robot.arduinoWrite(cmd);
        }

        private void errMessage(string msg)
        {
            MessageBox.Show(this, msg, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }

        #endregion

    }
}
