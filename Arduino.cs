using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

public class Arduino : MonoBehaviour
{
    public enum PortNumber
    {
        COM1, COM2, COM3, COM4, 
        COM5, COM6, COM7, COM8, 
        COM9, COM10, COM11, COM12, 
        COM13, COM14, COM15, COM16
    }

    //����� ��Ʈ, baud rate(��żӵ�)
    private SerialPort stream;

    [SerializeField]
    private PortNumber portNumber = PortNumber.COM1;
    [SerializeField]
    private string baudRate = "9600";

    
    void Start()
    {
        stream = new SerialPort(portNumber.ToString(), int.Parse(baudRate), Parity.None, 8, StopBits.One);

        stream.Open(); //Open the Serial Stream.
        stream.ReadTimeout = 5;
    }

    // Update is called once per frame
    void Update()
    {
        if (stream.IsOpen)
        {
            try
            {
                Test(stream.ReadByte());
            }
            catch(System.TimeoutException e)
            {
                Debug.Log(e);
                throw;
            }
            Debug.Log("�����");
        }
        else if (!stream.IsOpen)
        {
            Debug.Log("����ȵ�");
            stream.Open();
        }
    }

    void Test(int Number)
    {
        Debug.Log(Number);
    }
    private void OnApplicationQuit()
    {
        stream.Close();
    }

}