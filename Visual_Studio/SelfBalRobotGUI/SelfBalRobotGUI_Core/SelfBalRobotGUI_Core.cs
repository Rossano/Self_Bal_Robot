using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Arduino;

namespace Arduino
{
    struct IMUData
    {
        float yaw;
        float roll;
        float pitch;
        int gyro_roll;

 /*       public IMUData()
        {
            yaw = 0;
            roll = 0;
            pitch = 0;
            gyro_roll = 0;
        }*/
    };

    [System.Runtime.InteropServices.GuidAttribute("B9BED6C8-5BBF-4A1F-BCF9-DE05313697C2")]
    public class SelfBalRobotGUI_Core: MyArduino
    {		

        #region Constants

        const int VECTOR_LENGHT = 4;
        const string request_data = "get_val";
        const string request_feedback = "contr_state";
        const string request_info = "info";
        const string set_fb_coeff = "contr_set";
        const string get_fb_coeff = "contr_get";
        const string get_ack = "get_ACK";
        const string move_robot = "move";
        const string turn_robot = "turn";
        const string toggle_controller = "cont_toggle";
        const int STOP = 0;
        const int FORWARD = 1;
        const int BACKWARD = 2;
        const int NO_TURN = 0;
        const int LEFT = 1;
        const int RIGHT = 2;

        #endregion

        #region Members

        private IMUData _imuData;
        public long timeStamp;
        public float[] _angles {get; private set;}
        public int gyro { get; private set; }
        public float[] _state;
        public float[] K;
        public float F;
        public bool _controllerActive;
        public int pwm;
        public enum moveRobot_type
        {
            _FORWARD,
            _BACKWARD,
            _STOP
        };
        public enum turnRobot_Type
        {
            _LEFT,
            _RIGHT,
            _NO_TURN
        };

        #endregion

        #region Constructor

        public SelfBalRobotGUI_Core(string name, int baud): base(name, baud)
        {
            _imuData = new IMUData();
            _angles = new float[3];
            gyro = 0;
            _controllerActive=false;
            _state = new float[VECTOR_LENGHT];
            K = new float[VECTOR_LENGHT];
            for (int i = 0; i < VECTOR_LENGHT; i++)
            {
                K[i] = 0.0F;
            }

            System.Threading.Thread.Sleep(3000);
            // Empty Buffer with MP6050 messages
            Queue<string> foo = getFIFOBuffer();
            for (int i = 0; i < foo.Count; i++)
            {
                Console.WriteLine(foo.Dequeue());
            }
        }

        #endregion

        #region Methods

        public string getInfo()
        {
            string foo;
            arduinoWrite(request_info);
            foo = getBuffer();
            return foo;
        }

        public void getIMUData()
        {
            try
            {
                arduinoWrite(request_data);
                //string foo = getBuffer();
                System.Threading.Thread.Sleep(100);
                string foo;
                Queue<string> buff=getFIFOBuffer();
                bool found = false;
                int count = buff.Count;
                int j = 0;
                do
                {
                    foo = buff.Dequeue();
                    if (foo.StartsWith(get_val_Id)) found = true;
                    else buff.Enqueue(foo);
                } while (!found && ++j < count);
                if (string.IsNullOrEmpty(foo) || !found) return;
                char[] separators = { ' ', ',', ':' };
                string[] tokens = foo.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                timeStamp = Convert.ToInt64(tokens[2]);
                for (int i = 0; i < 2; i++)
                {
                    _angles[i] = (float)Convert.ToDouble(tokens[i + 3]);
                }
                gyro = Convert.ToInt16(tokens[6]);
                F = (float)Convert.ToDouble(tokens[7]);
                pwm = Convert.ToInt16(tokens[8]);
                getControllerState();
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void getControllerState()
        {
            try
            {
                arduinoWrite(request_feedback);
                System.Threading.Thread.Sleep(100);
                Queue<string> buff = getFIFOBuffer();
                bool found = false;
                int count = buff.Count;
                if (count == 0) return;
                string foo = "";
                do
                {
                    foo = buff.Dequeue();
                    if (foo.Contains(cont_state_Id)) found = true;
                    else buff.Enqueue(foo);
                } while (!found && --count >= 0);
                //string foo = getBuffer();
                if (string.IsNullOrEmpty(foo) || !found) return;
                char[] separators = { ' ', ',', '\t', '\r', '\n' };
                string[] tokens = foo.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                //if (!tokens[0].StartsWith(cont_state_Id)) return;
                int offs = 0;
                while (!tokens[offs].StartsWith(cont_state_Id)) offs++;
                for (int i = 0; i < VECTOR_LENGHT; i++)
                {
                    _state[i] = (float)Convert.ToDouble(tokens[offs + i + 1]);
                }
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void setControllerFeedback(double k1, double k2, double k3, double k4)
        {
            try
            {
                K[0] = (float)k1;
                K[1] = (float)k2;
                K[2] = (float)k3;
                K[3] = (float)k4;
                string cmd = set_fb_coeff + " " + k1.ToString() + " " + k2.ToString() + " " + k3.ToString() + " " + k4.ToString();
                arduinoWrite(cmd);
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void getControllerFeedback()
        {
            try
            {
                arduinoWrite(get_fb_coeff);
                System.Threading.Thread.Sleep(100);
                Queue<string> buff = getFIFOBuffer();
                string foo;
                int count = buff.Count;
                bool found = false;
                do
                {
                    foo = buff.Dequeue();
                    if (foo.StartsWith(cont_get_Id)) found = true;
                    else buff.Enqueue(foo);
                } while (!found && --count >= 0);
                //string foo=getBuffer();
                //if (string.IsNullOrEmpty(foo)) return;
                if (count < 0) return;
                char[] separators = { ' ', ',', '\t', '\r', '\n' };
                string[] tokens = foo.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                //int i = 0;
                //foreach(string s in tokens)
                if (tokens[0].Contains("Error")) throw new Exception(foo);
                for (int i = 0; i < 4; i++ )
                {
                    K[i] = (float)Convert.ToDouble(tokens[i + 1]);
                }
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void controllerToggle(bool active)
        {
            try
            {
                
                if (active)
                {
	                _controllerActive = active;
                    arduinoWrite(toggle_controller + " 1");
                }
                else
                {
                    _controllerActive = active;                    arduinoWrite(toggle_controller + " 0");

                }
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public bool getACK()
        {
            try
            {
                arduinoWrite(get_ack);                
                for(int i=0; i<300;i++)
                {
                    string foo = getBuffer();
                    if (foo.Contains("ACK")) return true;
                    System.Threading.Thread.Sleep(100);
                }
                return false;
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void stopRobot()
        {
            try
            {
                arduinoWrite(move_robot + " " + STOP + " 0");
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void moveRobot(moveRobot_type dir, int pwm)
        {
            try
            {
                int i;
                if (dir == moveRobot_type._STOP) return;
                else if (dir == moveRobot_type._FORWARD) i = FORWARD;
                else i = BACKWARD;
                arduinoWrite(move_robot + " " + i.ToString() + " " + pwm.ToString());
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void noturingRobot()
        {
            try
            {
                arduinoWrite(turn_robot + " " + NO_TURN);
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void turnRobot(turnRobot_Type dir)
        {
            try
            {
                int i;
                if (dir == turnRobot_Type._NO_TURN) return;
                else if (dir == turnRobot_Type._LEFT) i = LEFT;
                else i = RIGHT;
                arduinoWrite(turn_robot + " " + i.ToString());
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        #endregion
    }
}
