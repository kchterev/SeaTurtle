' This program is able to communicate with the Max-i chip and most other serial devices by means of a COM port. The
'   program has been designed for very high-speed operation with maximum efficiency as Max-i goes up to 1.843 Mbit/s
'     and is able to transfer up to 10,000 telegrams per second where several hundreds may be passed to the PC.
'       Because of this, not everything is made quite by-the-book. For example, the program uses global buffers
'         (usually not recommended for object oriented programming) and Invoke instead of BeginInvoke.
' The input to the transmitter field may be any mix between hexadecimal numbers with an even number of digits and
'   ASCII characters embedded in quotation marks like e.g. "UU" 77 """A" 66 55FF => 55 55 77 22 41 66 55 FF.
'     Note that ASCII " is specified as "" as in Visual Basic. In case of hexadecimal numbers with 4 or more
'       digits, the number is transmitted with the big-endian model, meaning that the most significant part is
'         transmitted first.
' To make it easier to separate transmissions and receptions, the program appends a "TX" to the received textbox each
'   time you transmit anything.
' The maximum length of each telegram is 2048 bytes. If longer telegrams are needed, the size of the two arrays
'   RXArray and TXArray must be increased.
'
' Innovatic, Carsten Kanstrup. February 19th 2010

Imports System
Imports System.IO.Ports
Imports System.Threading
Imports System.Threading.Thread
Imports System.Text
Imports System.IO

' Imports the System, System.IO.Ports and System.Threading.Thread namespaces so that e.g.
'   System.IO.Ports.SerialPort may just be written as SerialPort, and System.EventArgs may just be
'     written as EventArgs.

Public Class ORCATester

    Dim SpaceCount As Byte = 0
    Dim LookUpTable As String = "0123456789ABCDEF"
    Dim RXArray(2047) As Char ' Text buffer. Must be global to be accessible from more threads.
    Dim RXBArray(2047) As Byte ' Data buffer. Must be global to be accessible from more threads.
    Dim MotorID As Byte
    Dim RxRPM, RxCurrent, RxTemp As Integer
    Dim RxRPM1, RxCurrent1, RxTemp1 As Integer
    Dim RxRPM2, RxCurrent2, RxTemp2 As Integer
    Dim RxRPM3, RxCurrent3, RxTemp3 As Integer
    Dim TxDataLen As Integer
    Dim RxDataLen As Integer

    Dim MIN_FLOAT As Single = 0.0000000001
    Dim MAX_FLOAT As Single = 10000000000.0

    Dim ax As Single
    Dim ay As Single
    Dim az As Single

    Dim mgx As Single
    Dim mgy As Single
    Dim mgz As Single

    Dim gx As Single
    Dim gy As Single
    Dim gz As Single

    Dim hh As Double

    Dim objStreamWriter As StreamWriter

    ' Make a new System.IO.Ports.SerialPort instance, which is able to fire events.
    Dim WithEvents COMPort As New SerialPort


    Private WithEvents joystick1 As Joystick

    Public Sub CompasCalc()
        If MotorID = 7 Then
            If LIN.Checked = True Then

                ax = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 6)
                ay = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 8)
                az = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 10)

                'mgx = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 12)
                mgx = RXBArray(TxDataLen + 3 + 12) * 256 + RXBArray(TxDataLen + 3 + 13)

                mgy = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 14)
                mgz = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 16)

                gx = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 0)
                gy = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 2)
                gz = BitConverter.ToInt16(RXBArray, TxDataLen + 3 + 4)



            Else
                ax = BitConverter.ToInt16(RXBArray, 6)
                ay = BitConverter.ToInt16(RXBArray, 8)
                az = BitConverter.ToInt16(RXBArray, 10)

                ' mgx = BitConverter.ToInt16(RXBArray, 12)
                mgy = BitConverter.ToInt16(RXBArray, 14)
                mgz = BitConverter.ToInt16(RXBArray, 16)

                gx = BitConverter.ToInt16(RXBArray, 0)
                gy = BitConverter.ToInt16(RXBArray, 2)
                gz = BitConverter.ToInt16(RXBArray, 4)


            End If

            'hh = Math.Atan2(mgy, mgx) * 180 / Math.PI

            'If (hh < 0.0) Then
            'hh = hh + 360.0
            '
            'End If

            'X.Text = mgx
            'Y.Text = mgy
            'Compass.Text = hh

            X.Text = ax
            Y.Text = gx
            Compass.Text = mgx / 10

        End If

    End Sub

    Public Sub Receiver(ByVal sender As Object, ByVal e As SerialDataReceivedEventArgs) Handles COMPort.DataReceived

        If COMPort.BytesToRead >= RxDataLen Then

            Dim i As Integer
            ' Do
            '----- Start of communication protocol handling -----------------------------------------------------------
            ' The code between the two lines does the communication protocol. In this case, it simply emties the
            '   receive buffer and converts it to text, but for all practical applications, you must replace this part
            '     with a code, which can collect one entire telegram by searching for the telegram
            '       delimiter/termination. In case of a simple ASCII protocol, you may just use ReadLine and receive
            '         in a global string instead of a byte array.
            ' Because this routine runs on a thread pool thread, it does not block the UI, so if you have any data
            '   convertion, encryption, expansion, error detection, error correction etc. to do, do it here.

            'COMPort.Read(RXBArray, 0, RxDataLen)

            i = 0

            Do

                RXBArray(i) = COMPort.ReadByte()
                i = i + 1

            Loop Until (i = RxDataLen)

            If LIN.Checked = True Then
                RxRPM = RXBArray(TxDataLen + 3) * 256 + RXBArray(TxDataLen + 4)
                RxCurrent = RXBArray(TxDataLen + 5) * 256 + RXBArray(TxDataLen + 6)
                RxTemp = RXBArray(TxDataLen + 7)
            Else
                RxRPM = RXBArray(0) * 256 + RXBArray(1)
                RxCurrent = RXBArray(2) * 256 + RXBArray(3)
                RxTemp = RXBArray(4)
            End If

            If RxTemp > 128 Then
                RxTemp = RxTemp - 256
            End If

            If MotorID = 1 Then
                RxRPM1 = RxRPM
                RxCurrent1 = RxCurrent
                RxTemp1 = RxTemp
            End If
            If MotorID = 2 Then
                RxRPM2 = RxRPM
                RxCurrent2 = RxCurrent
                RxTemp2 = RxTemp
            End If
            If MotorID = 3 Then
                RxRPM3 = RxRPM
                RxCurrent3 = RxCurrent
                RxTemp3 = RxTemp
            End If

            If LIN.Checked = True Then

                For i = TxDataLen + 3 To RxDataLen - 1  'LIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                    RXArray(3 * (i - TxDataLen - 3)) = LookUpTable(RXBArray(i) >> 4) ' Convert each byte to two hexadecimal characters
                    RXArray(3 * (i - TxDataLen - 3) + 1) = LookUpTable(RXBArray(i) And 15)

                    RXArray(3 * (i - TxDataLen - 3) + 2) = " "
                Next

                RXArray(3 * (i - TxDataLen - 3)) = Chr(13)
                RXArray(3 * (i - TxDataLen - 3) + 1) = Chr(10)

            Else

                For i = 0 To RxDataLen - 1              'RS232!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    RXArray(3 * i) = LookUpTable(RXBArray(i) >> 4) ' Convert each byte to two hexadecimal characters
                    RXArray(3 * i + 1) = LookUpTable(RXBArray(i) And 15)

                    RXArray(3 * i + 2) = " "
                Next

                RXArray(3 * i) = Chr(13)
                RXArray(3 * i + 1) = Chr(10)

            End If

            '----- End of communication protocol handling -------------------------------------------------------------
            Me.Invoke(New MethodInvoker(AddressOf Display)) ' Start "Display" on the UI thread

        End If
    End Sub

    ' Text display routine, which appends the received string to any text in the Received TextBox.

    Private Sub Display()
        If LIN.Checked = True Then
            Received.AppendText(New String(RXArray, 0, (RxDataLen - TxDataLen - 2) * 3 - 1)) 'LIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Else
            Received.AppendText(New String(RXArray, 0, (RxDataLen) * 3 + 2))  'RS232!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        End If
        CompasCalc()

    End Sub

    ' Transmitter subroutine.

    Private Sub Transmitter(ByVal sender As Object, ByVal e As EventArgs) Handles SendButton.Click
        'Received.AppendText("TX" & vbCrLf)        ' Switch to a new line after every transmission
        SpaceCount = 0
        Dim TextString As String
        Dim TXArray(2047) As Byte
        Dim I As Integer
        Dim J As Integer = 0
        Dim Ascii As Boolean = False
        Dim Quote As Boolean = False
        Dim Temp As Boolean
        Dim Second As Boolean = False
        Dim TXByte As Byte = 0
        Dim CharByte As Byte
        Dim ChkSum As Byte


        If COMPort.IsOpen Then
            TextString = Transmitted.Text
            For I = 0 To TextString.Length - 1
                CharByte = Asc(TextString.Chars(I))
                If CharByte = 34 Then ' If " Then
                    Temp = Ascii
                    Ascii = Ascii Or Quote
                    Quote = Not (Temp And Quote)
                Else
                    Ascii = Ascii Xor Quote
                    Quote = False
                End If
                If Not Quote Then
                    If Ascii Then
                        TXArray(J) = CharByte
                        J = J + 1
                    Else
                        If (CharByte <> 32) And (CharByte <> 10) And (CharByte <> 13) Then ' Skip spaces, LF and CR
                            CharByte = (CharByte - 48) And 31 ' And 31 makes it case insensitive
                            If CharByte > 16 Then
                                CharByte = CharByte - 7
                            End If
                            If Second Then
                                TXArray(J) = TXByte + CharByte
                                Second = False
                                J = J + 1
                            Else
                                TXByte = CharByte << 4
                                Second = True
                            End If
                        End If
                    End If
                End If
            Next
            'Try



            COMPort.BreakState = True
            Sleep((11000 / COMPort.BaudRate) + 15)
            ' Min. 11 bit delay (startbit, 8 data bits, parity bit, stopbit). 10 mS have been added to ensure that
            '   the delay is always active. If the delay time is less than the task switching time, the two BreakState
            '     instructions may be activated immediately after each other ! On a multiprocessor system, you must
            '       add 15 mS instead.
            COMPort.BreakState = False

            ChkSum = 0
            ChkSum = ChkSum Xor TXArray(0)
            'COMPort.Parity = Parity.Mark
            COMPort.Write(TXArray, 0, 1)
            'Catch ex As Exception
            'MsgBox(ex.Message & "  Check CTS signal or set Flow Control to None.")
            'End Try
            'Sleep(10)
            'Try
            I = 1
            While I < J
                ChkSum = ChkSum Xor TXArray(I)
                I = I + 1
            End While
            TXArray(J) = ChkSum

            'COMPort.Parity = Parity.Space
            COMPort.Write(TXArray, 1, J)

            'Catch ex As Exception
            'MsgBox(ex.Message & "  Check CTS signal or set Flow Control to None.")
            'End Try

            COMPort.DiscardInBuffer()
        Else
            MsgBox("COM port is closed. Please select a COM port")
        End If
    End Sub

    ' This subroutine handles a change of COM ports.

    Private Sub PortSelection(ByVal sender As Object, ByVal e As EventArgs) Handles COMPortsBox.SelectedIndexChanged

        If COMPort.IsOpen Then
            COMPort.RtsEnable = False
            COMPort.DtrEnable = False
            ClosePort()
            ' NOTE. Because of the use of Invoke, the port should normally be closed from a different thread, see
            '   ORCATesterClosing, but to ensure that the following delay works, this is not done in this case.
            ' Empty the message queue before the UI thread goes to sleep to update the modem lamps and the ComboBox.
            Application.DoEvents()
            Sleep(200)                      ' Wait 0.2 second for port to close as this does not happen immediately.
        End If
        COMPort.PortName = COMPortsBox.Text
        COMPort.BaudRate = 19200            ' Default for Max-i: 19200 bit/s, 8 data bits, no parity, 1 stop bit
        COMPort.WriteTimeout = 2000         ' Max time to wait for CTS = 2 sec.
        COMPort.ReadBufferSize = 16384      'Necessary buffer size for 16C950 UART at 921.6 kbit/s
        COMPort.DataBits = 8
        COMPort.Parity = Parity.None
        COMPort.Handshake = Handshake.None
        Try
            COMPort.Open()
            StartTimer.Enabled = True
            SendConfigButton.Enabled = True
            GetStatusButton.Enabled = True
            GetStatusButton.Enabled = True
            BreakButton.Enabled = True
            NextID.Enabled = True
            GetConfigButton.Enabled = True

        Catch ex As Exception
            MsgBox(ex.Message)
        End Try


    End Sub

    ' This subroutine is activated when the form is loaded. It does all the basic initializations.
    ' Note! Baud Rates above 115.2 KBaud requires a special UART like e.g. 16C650, 16C750, 16C850 or 16C950 and
    '   a special driver.
    ' RTS/CTS hardware flow control is necessary for Max-i communication. Note that the standard UART 16C550
    '   has no hardware flow control - except for UART's from Texas Instruments.
    ' The minimum FIFO size is calculated on the basis of a 1.4 mS response time, which is the absolute minimum
    '   for Windows applications.

    Private Sub ORCATesterLoad(ByVal sender As Object, ByVal e As EventArgs) Handles MyBase.Load
        For Each COMString As String In My.Computer.Ports.SerialPortNames ' Load all available COM ports.
            COMPortsBox.Items.Add(COMString)
        Next

        ' Here we create the Joystick object. You must pass Me - which refers to this Form,
        ' and 0 - which is the Joystick id.
        joystick1 = New Joystick(Me, 0)

        COMPortsBox.Sorted = True
        BaudRateBox.Items.Add("110")
        BaudRateBox.Items.Add("300")
        BaudRateBox.Items.Add("600")
        BaudRateBox.Items.Add("1200")
        BaudRateBox.Items.Add("1800")
        BaudRateBox.Items.Add("2400")
        BaudRateBox.Items.Add("4800")
        BaudRateBox.Items.Add("7200")
        BaudRateBox.Items.Add("9600")
        BaudRateBox.Items.Add("14400")
        BaudRateBox.Items.Add("19200")      ' Min. FIFO size 3 Bytes (8030 or 8530)
        BaudRateBox.Items.Add("38400")
        BaudRateBox.Items.Add("57600")      ' Min. FIFO size 8 bytes
        BaudRateBox.Items.Add("115200")     ' Min. FIFO size 16 bytes (16C550)
        BaudRateBox.Items.Add("230400")     ' Min. FIFO size 32 bytes (16C650)
        BaudRateBox.Items.Add("460800")     ' Min. FIFO size 64 bytes (16C750)
        BaudRateBox.Items.Add("921600")     ' Min. FIFO size 128 bytes (16C850 or 16C950)

        ID_Box.Text = 1
        MotorID = 1

        Dim editconf = New EditConfig()

        objStreamWriter = New StreamWriter("D:\tmp\DataLog.txt")
        objStreamWriter.Close()


    End Sub


    ' This subroutine is used to close the COM port. Because the program uses Invoke instead of BeginInvoke, this
    '   routine is usually called on a separate (new) thread to prevent a close-down deadlock.

    Private Sub ClosePort()
        If COMPort.IsOpen Then COMPort.Close()
    End Sub

    ' This subroutine is activated when the form is closed. It closes the COM port. Without such a close command,
    '   the garbage collector may close the COM port while it is still in use!

    Private Sub ORCATesterClosing(ByVal sender As Object, ByVal e As ComponentModel.CancelEventArgs) Handles MyBase.Closing
        If MessageBox.Show("Do you really want to close the window", "", MessageBoxButtons.YesNo) = Windows.Forms.DialogResult.No Then
            e.Cancel = True
        Else
            ' Close COM port on a new thread when the form is terminated with [X]
            Dim t As New Thread(AddressOf ClosePort)
            t.Start()
            objStreamWriter.Close()
        End If
    End Sub

    ' This subroutine clears the Received TextBox for received bytes.

    Private Sub ClearReceivedText(ByVal sender As Object, ByVal e As EventArgs) Handles ClearButton.Click
        Received.Text = ""
        SpaceCount = 0
    End Sub

    ' This subroutines inserts a Break condition.

    Private Sub ResetControler(ByVal sender As Object, ByVal e As EventArgs) Handles BreakButton.Click

        ''>>>If COMPort.IsOpen Then
        '    For i = 1 To 3

        '        COMPort.BreakState = True
        '        Sleep((11000 / COMPort.BaudRate) + 15)
        '    ' >>>Min. 11 bit delay (startbit, 8 data bits, parity bit, stopbit). 10 mS have been added to ensure that
        '    ' >>>>  the delay is always active. If the delay time is less than the task switching time, the two BreakState
        '    ' >>>    instructions may be activated immediately after each other ! On a multiprocessor system, you must
        '    ' >>>     add 15 mS instead.
        '        COMPort.BreakState = False
        '        Sleep((11000 / COMPort.BaudRate) + 15)
        '    Next

        'Else
        '    MsgBox("No COM Port Selected")
        'End If

        Dim TXArray(2047) As Byte
        Dim chkSum As Byte = 0

        SyncLock COMPort

            TxDataLen = 2                   ' Total frame length -9 bytes. Kp, Ki and Kd - 3 x 2 = 6  + 1 DI + 1 LEN + 1 CHKSUM
            TXArray(0) = MotorID '+ 128      ' Header is  Motor ID with MSB set to indicate a command sequense
            TXArray(1) = 255                ' 0xFF for resetting the system

            If COMPort.IsOpen Then

                'send a break signal
                COMPort.BreakState = True
                Sleep((11000 / COMPort.BaudRate) + 15)
                ' Min. 11 bit delay (startbit, 8 data bits, parity bit, stopbit). 10 mS have been added to ensure that
                '   the delay is always active. If the delay time is less than the task switching time, the two BreakState
                '     instructions may be activated immediately after each other ! On a multiprocessor system, you must
                '       add 15 mS instead.
                COMPort.BreakState = False

                chkSum = 0
                'calculate the check sum over the whole frame
                chkSum = chkSum Xor TXArray(0)
                chkSum = chkSum Xor TXArray(1)

                TXArray(TxDataLen) = chkSum         ' store the check sum in the transmit buffer

                COMPort.Write(TXArray, 0, 3)        ' Transmit the frame
            Else
                MsgBox("No COM Port Selected")
            End If

        End SyncLock



    End Sub

    ' This subroutine handles a change in Baud Rate.

    Private Sub BaudRateSelection(ByVal sender As Object, ByVal e As EventArgs) Handles BaudRateBox.SelectedIndexChanged
        COMPort.BaudRate = CInt(BaudRateBox.Text)

        If COMPort.IsOpen Then
            'COMPort.BreakState = True
            'Sleep((11000 / COMPort.BaudRate) + 20)
            ' Min. 11 bit delay (startbit, 8 data bits, parity bit, stopbit). 10 mS have been added to ensure that
            '   the delay is always active. If the delay time is less than the task switching time, the two BreakState
            '     instructions may be activated immediately after each other ! On a multiprocessor system, you must
            '       add 15 mS instead.
            'COMPort.BreakState = False
            'Sleep(10)
            'COMPort.Write("U", 0, 1)
            'Sleep(10)

        End If
    End Sub




    ' This subroutine saves the content of the Received TextBox

    Private Sub SaveText(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim SaveFileDialog1 As New SaveFileDialog()
        SaveFileDialog1.Filter = "Text Files (*.txt)|*.txt"
        SaveFileDialog1.Title = "Save Received As"
        If SaveFileDialog1.ShowDialog() = System.Windows.Forms.DialogResult.OK _
            And SaveFileDialog1.FileName.Length > 0 Then
            My.Computer.FileSystem.WriteAllText(SaveFileDialog1.FileName, Received.Text, False) ' Overwrite file
        End If
    End Sub

    Private Sub Motor1SP_Scroll(sender As Object, e As EventArgs) Handles Motor1SP.Scroll
        SP1.Text = Motor1SP.Value
    End Sub

    Private Sub Motor2SP_Scroll(sender As Object, e As EventArgs) Handles Motor2SP.Scroll
        SP2.Text = Motor2SP.Value
    End Sub


    Private Sub Motor3SP_Scroll(sender As Object, e As EventArgs) Handles Motor3SP.Scroll
        SP3.Text = Motor3SP.Value
    End Sub

    Private Sub Motor4SP_Scroll(sender As Object, e As EventArgs) Handles Motor4SP.Scroll
        SP4.Text = Motor4SP.Value
    End Sub


    Private Sub BuildAndSend()

        Dim TXArray(2047) As Byte
        Dim chkSum As Byte = 0
        Dim tmp As Integer
        Dim I As Byte

        SyncLock COMPort

            TxDataLen = 26                  ' Total frame length - 27 bytes. 2 x Number of Motors/Servos = 24 data bytes, 1 byte Header, 1 byte length, 1 byte checksum
            TXArray(0) = MotorID            ' Header is either the Motor ID, 0xFF for broadcast or Motor ID with MSB set to indicate a command sequense
            TXArray(1) = TxDataLen          ' Length is only the databytes + 1 check sum byte

            'Prepare Motor 1 parameters
            If (Motor1SP.Value < 0) Then
                tmp = 65536 + Motor1SP.Value    'if negative calculate the complimentary value
            Else
                tmp = Motor1SP.Value
            End If

            TXArray(2) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(3) = tmp And 255

            'Prepare Motor 2 parameters
            If (Motor2SP.Value < 0) Then
                tmp = 65536 + Motor2SP.Value    'if negative calculate the complimentary value
            Else
                tmp = Motor2SP.Value
            End If

            TXArray(4) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(5) = tmp And 255

            'Prepare Motor 3 parameters
            If (Motor3SP.Value < 0) Then
                tmp = 65536 + Motor3SP.Value    'if negative calculate the complimentary value
            Else
                tmp = Motor3SP.Value
            End If

            TXArray(6) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(7) = tmp And 255

            'Prepare Motor 4 parameters
            TXArray(8) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(9) = tmp And 255

            'Prepare Motor 5 parameters
            TXArray(10) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(11) = tmp And 255

            'Prepare Motor 6 parameters
            TXArray(12) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(13) = tmp And 255

            'Servo part

            'Prepare Servo - ID = 7 parameters

            Dim SRV1, SRV2, SRV3, SRV4, SRV5, SRV6 As Integer

            SRV1 = Motor4SP.Value
            SRV2 = Motor4SP.Value
            SRV3 = Motor4SP.Value
            SRV4 = Motor4SP.Value
            SRV5 = Motor4SP.Value
            SRV6 = Motor4SP.Value

            If (SRV1 < 0) Then
                tmp = 65536 + Motor4SP.Value    'if negative calculate the complimentary value
            Else
                tmp = Motor4SP.Value
            End If

            TXArray(14) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(15) = tmp And 255


            If (SRV2 < 0) Then
                tmp = -SRV1 'tmp = 65536 + SRV2    'if negative calculate the complimentary value
            Else
                tmp = SRV2
            End If



            TXArray(16) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(17) = tmp And 255

            If (SRV3 < 0) Then
                tmp = -SRV3 'tmp = 65536 + SRV3    'if negative calculate the complimentary value
            Else
                tmp = SRV3
            End If


            TXArray(18) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(19) = tmp And 255

            If (SRV4 < 0) Then
                tmp = 65536 + SRV4    'if negative calculate the complimentary value
            Else
                tmp = SRV4
            End If

            TXArray(20) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(21) = tmp And 255

            If (SRV5 < 0) Then
                tmp = SRV5 + 32768    'if negative calculate the complimentary value
            Else
                tmp = SRV5 + 32768
            End If

            TXArray(22) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(23) = tmp And 255

            If (SRV6 < 0) Then
                tmp = SRV6 + 32768    'if negative calculate the complimentary value
            Else
                tmp = SRV6 + 32768
            End If

            TXArray(24) = Int(tmp / 256)         ' calculate and store 2 bytes representing the setpoint value
            TXArray(25) = tmp And 255

            If COMPort.IsOpen Then

                chkSum = 0
                I = 0
                While I < TxDataLen + 2              'calculate the check sum over the whole frame
                    chkSum = chkSum Xor TXArray(I)
                    I = I + 1
                End While

                TXArray(I) = chkSum         ' store the check sum in the transmit buffer

                COMPort.DiscardInBuffer()
                COMPort.DiscardOutBuffer()

                COMPort.Write(TXArray, 0, TxDataLen + 3)        ' Transmit the frame



                If MotorID <> 7 Then
                    If LIN.Checked = True Then
                        RxDataLen = TxDataLen + 3 + 6
                    Else
                        RxDataLen = 6
                    End If
                Else
                    If LIN.Checked = True Then
                        RxDataLen = TxDataLen + 3 + 31     'LIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    Else
                        RxDataLen = 31                     'RS232!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    End If
                End If

            Else
                MsgBox("No COM Port Selected")
            End If

        End SyncLock


    End Sub


    Private Sub TextBox1_KeyPress(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyPressEventArgs) Handles SP1.KeyPress
        If Asc(e.KeyChar) = 13 Then
            Motor1SP.Value = CInt(SP1.Text)
        End If
    End Sub

    Private Sub TextBox2_KeyPress(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyPressEventArgs) Handles SP2.KeyPress
        If Asc(e.KeyChar) = 13 Then
            Motor2SP.Value = CInt(SP2.Text)
        End If
    End Sub

    Private Sub TextBox3_KeyPress(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyPressEventArgs) Handles SP3.KeyPress
        If Asc(e.KeyChar) = 13 Then
            Motor3SP.Value = CInt(SP3.Text)
        End If
    End Sub

    Private Sub TextBox4_KeyPress(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyPressEventArgs) Handles SP4.KeyPress
        If Asc(e.KeyChar) = 13 Then
            Motor4SP.Value = CInt(SP4.Text)
        End If
    End Sub
    Private Sub TextBox5_KeyPress(ByVal sender As Object, ByVal e As System.Windows.Forms.KeyPressEventArgs) Handles ID_Box.KeyPress
        If Asc(e.KeyChar) = 13 Then
            MotorID = CInt(ID_Box.Text)
        End If
    End Sub



    '''''''''''''''''''''''''''JOYSTICK

    ' And now we have the four events that belong to joystick1.


    Private Sub joystick1_Joy() Handles joystick1.Joy

        If COMPort.IsOpen Then
            Motor1SP.Value = (32768 - joystick1.p.YPos) * 11 * 850 / 32768 '(volts * Kv - max RPM )
            SP1.Text = Motor1SP.Value

            Motor2SP.Value = (joystick1.p.XPos - 32768) * 11 * 850 / 32768
            SP2.Text = Motor2SP.Value
        End If

    End Sub

    ' Private Sub joystick1_buttonPressed() Handles joystick1.buttonPressed
    ' TODO: Replace this so that it plays a sound instead.
    '    Me.Text = joystick1.b1
    'End Sub

    Private Sub Timer1_Tick(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Timer1.Tick


        RPM1.Text = RxRPM1
        Current1.Text = RxCurrent1
        Temp1.Text = RxTemp1

        RPM2.Text = RxRPM2
        Current2.Text = RxCurrent2
        Temp2.Text = RxTemp2

        RPM3.Text = RxRPM3
        Current3.Text = RxCurrent3
        Temp3.Text = RxTemp3

        MotorID = MotorID + 1
        ' MotorID = 7 'DEBUG!!!!!!!!!!!!!!

        If MotorID > 7 Then
            MotorID = 1
        End If

        ID_Box.Text = MotorID

        BuildAndSend()

    End Sub

    Private Sub ID_Box_TextChanged(sender As Object, e As EventArgs) Handles ID_Box.TextChanged
        MotorID = ID_Box.Text
    End Sub

    Private Sub TransmitConfiguration(sender As Object, e As EventArgs) Handles SendConfigButton.Click

        Dim TXArray(2048) As Byte
        Dim ParsedFile(2048)
        Dim ParamListStr(1024)
        Dim ParamListBin(1024)
        Dim chkSum As Byte = 0
        Dim I, J As Integer


        SyncLock COMPort


            Using MyReader As New Microsoft.VisualBasic.FileIO.TextFieldParser("config.ini")
                MyReader.TextFieldType = FileIO.FieldType.Delimited
                MyReader.SetDelimiters(",")
                Dim currentRow As String()
                Dim currentField As String

                I = 0
                While Not MyReader.EndOfData()
                    Try
                        currentRow = MyReader.ReadFields()

                        For Each currentField In currentRow
                            ParsedFile(I) = currentField
                            I = I + 1
                        Next

                    Catch ex As Microsoft.VisualBasic.FileIO.MalformedLineException
                        MsgBox("Line " & ex.Message & "is not valid and will be skipped.")
                    End Try
                End While
            End Using

            For J = 0 To I / 2 + 1
                ParamListStr(J) = ParsedFile(2 * J + 1)
                ParamListBin(J) = Convert.ToInt32(ParamListStr(J), 10)
            Next J

            '1. uint8_t	board_id;	//Board identifier. // MUST BE UNIQUE FOR EACH BOARD !!!!!!!!
            TXArray(2) = ParamListBin(0)

            '2. uint8_t allign_duty;//This duty cycle defines the momentum the motor will apply when started.
            TXArray(3) = ParamListBin(1)

            '3. uint8_t ramp_duty;//The RAMP_DUTY defines the momentum of the ramping stage.
            TXArray(4) = ParamListBin(2)

            '4. uint8_t delta_sp;//Defines the acceleration. The setpoint is increase by delta_sp every fast event.
            TXArray(5) = ParamListBin(3)

            '5. uint16_t stop_delay;			//Defines the time in STOP state.
            TXArray(6) = ParamListBin(4) And 255
            TXArray(7) = (ParamListBin(4) And 65280) / 256

            '6. uint16_t align_delay;			//Defines the time in ALLIGN state.
            TXArray(8) = ParamListBin(5) And 255
            TXArray(9) = (ParamListBin(5) And 65280) / 256


            '7. uint16_t start_comm_time;		//Initial commutatation time.
            TXArray(10) = ParamListBin(6) And 255
            TXArray(11) = (ParamListBin(6) And 65280) / 256

            '8. uint16_t blanking_time;		//Zero Crossing check Blanking Time.
            TXArray(12) = ParamListBin(7) And 255
            TXArray(13) = (ParamListBin(7) And 65280) / 256

            '9. uint16_t zc_correction;	//Zero Crossing Correction. Due to lag etc, the ZC moment may have to be corrected.
            TXArray(14) = ParamListBin(8)

            '10. uint8_t acc_zc_gain;//Zero Crossing error multiplier.If the error is positive.
            TXArray(15) = ParamListBin(9)

            '11. uint8_t dec_zc_gain;//Zero Crossing error multiplier.If the error is negative.
            TXArray(16) = ParamListBin(10)

            '12. uint8_t zc_count;		//Counts the Zero Crossings close to 1/2 comm_time.
            TXArray(17) = ParamListBin(11)

            '13. uint8_t zc_stall_count;		//Counts the low delta Phase Voltage events.
            TXArray(18) = ParamListBin(12)

            '14. uint8_t stall_phase_voltage;	//Delta Phase Voltage threshold.
            TXArray(19) = ParamListBin(13)

            '15. uint8_t rpm_IIR_A;				//Filter params for RPM calculation.
            TXArray(20) = ParamListBin(14)

            '16. uint8_t rpm_IIR_B;
            TXArray(21) = ParamListBin(15)

            '17. uint8_t current_IIR_A;			//Filter params for current calculation.
            TXArray(22) = ParamListBin(16)

            '18. uint8_t current_IIR_B;
            TXArray(23) = ParamListBin(17)

            '19. uint8_t comm_IIR_A;		//Filter params for communication time calculation.
            TXArray(24) = ParamListBin(18)

            '20. uint8_t comm_IIR_B;
            TXArray(25) = ParamListBin(19)

            '21. uint32_t max_current;			//Motor max allowed current in milliamps.
            TXArray(26) = ParamListBin(20) And 255
            TXArray(27) = (ParamListBin(20) And 65280) / 256
            TXArray(28) = (ParamListBin(20) And 16711680) / 65536
            TXArray(29) = (ParamListBin(20) And 4278190080) / 16777216

            '22. uint8_t max_temp;				//Motor max allowed temperature.
            TXArray(30) = ParamListBin(21) And 255
            TXArray(31) = (ParamListBin(21) And 65280) / 256

            '23. uint16_t pid_kp;//PID proportional param. If 0 - the PID regulator is disabled. Open loop control.
            TXArray(32) = ParamListBin(22) And 255
            TXArray(33) = (ParamListBin(22) And 65280) / 256

            '24. uint16_t pid_ki;				//PID integral param.
            TXArray(34) = ParamListBin(23) And 255
            TXArray(35) = (ParamListBin(23) And 65280) / 256

            '25. uint16_t pid_kd;				//PID differential param.
            TXArray(36) = ParamListBin(24) And 255
            TXArray(37) = (ParamListBin(24) And 65280) / 256

            '26. uint16_t min_rpm;//Minimum allowed RPM. Bellow this RPM the SP is set to 0.
            TXArray(38) = ParamListBin(25) And 255
            TXArray(39) = (ParamListBin(25) And 65280) / 256

            '27. uint16_t p1;					//Some spare configuration parameters. RFU
            TXArray(40) = ParamListBin(26) And 255
            TXArray(41) = (ParamListBin(26) And 65280) / 256

            '28. uint16_t p2;					//RFU
            TXArray(42) = ParamListBin(27) And 255
            TXArray(43) = (ParamListBin(27) And 65280) / 256

            '29. uint16_t p3;					//RFU
            TXArray(44) = ParamListBin(28) And 255
            TXArray(45) = (ParamListBin(28) And 65280) / 256

            '30. uint16_t p4;					//RFU
            TXArray(46) = ParamListBin(29) And 255
            TXArray(47) = (ParamListBin(29) And 65280) / 256

            '31. uint16_t p5;					//RFU
            TXArray(48) = ParamListBin(30) And 255
            TXArray(49) = (ParamListBin(30) And 65280) / 256

            '32. uint16_t p6;					//RFU
            TXArray(50) = ParamListBin(31) And 255
            TXArray(51) = (ParamListBin(31) And 65280) / 256

            '33. uint16_t p7;					//RFU
            TXArray(52) = ParamListBin(32) And 255
            TXArray(53) = (ParamListBin(32) And 65280) / 256

            '34. uint16_t p8;					//RFU
            TXArray(54) = ParamListBin(33) And 255
            TXArray(55) = (ParamListBin(33) And 65280) / 256


            chkSum = 0
            For I = 2 To 55
                chkSum = chkSum Xor TXArray(I)
            Next

            '35.  uint8_t	checksum;				//XOR of all the params
            TXArray(56) = chkSum


            TxDataLen = 55                  ' 56 pram bytes  
            TXArray(0) = MotorID + 128      ' Header is  Motor ID with MSB set to indicate a command sequense
            TXArray(1) = TxDataLen          ' Length is the databytes + 1 check sum byte

            If COMPort.IsOpen Then

                chkSum = 0
                I = 0
                While I < TxDataLen + 2                'calculate the check sum over the whole frame
                    chkSum = chkSum Xor TXArray(I)
                    I = I + 1
                End While

                TXArray(I) = chkSum         ' store the check sum in the transmit buffer

                COMPort.DiscardInBuffer()
                COMPort.DiscardOutBuffer()

                RxDataLen = 256

                COMPort.Write(TXArray, 0, TxDataLen + 3)            ' Transmit the remaining frame - data bytes + header + checksum
            Else
                MsgBox("No COM Port Selected")
            End If


        End SyncLock
    End Sub

    Private Sub StartTimer_Click(sender As Object, e As EventArgs) Handles StartTimer.Click
        Timer1.Enabled = True
        SendConfigButton.Enabled = False
        GetStatusButton.Enabled = False
        NextID.Enabled = False
        GetStatusButton.Enabled = False
        StartTimer.Enabled = False
        StopTimer.Enabled = True
        BreakButton.Enabled = False
        GetConfigButton.Enabled = False

    End Sub

    Private Sub StopTimer_Click(sender As Object, e As EventArgs) Handles StopTimer.Click
        Timer1.Enabled = False
        GetStatusButton.Enabled = True
        SendConfigButton.Enabled = True
        NextID.Enabled = True
        GetStatusButton.Enabled = True
        StartTimer.Enabled = True
        StopTimer.Enabled = False
        BreakButton.Enabled = True
        GetConfigButton.Enabled = True

        objStreamWriter.Close()

    End Sub

    Private Sub NextID_Click(sender As Object, e As EventArgs) Handles NextID.Click



        RPM1.Text = RxRPM1
        Current1.Text = RxCurrent1
        Temp1.Text = RxTemp1

        RPM2.Text = RxRPM2
        Current2.Text = RxCurrent2
        Temp2.Text = RxTemp2

        RPM3.Text = RxRPM3
        Current3.Text = RxCurrent3
        Temp3.Text = RxTemp3


        MotorID = MotorID + 1
        If MotorID > 7 Then
            MotorID = 1
        End If

        ID_Box.Text = MotorID

        BuildAndSend()
    End Sub

    Private Sub GetStatusButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles GetStatusButton.Click

        ID_Box.Text = MotorID

        BuildAndSend()

    End Sub

    Private Sub ExitButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles ExitButton.Click
        If MsgBox("Are you sure you want to exit?", MsgBoxStyle.OkCancel) = MsgBoxResult.Ok Then
            objStreamWriter.Close()
            Application.Exit()
        End If
    End Sub

    Private Sub EditConfgiButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles EditConfgiButton.Click
        EditConfig.Show()
    End Sub

    Private Sub GetConfigButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles GetConfigButton.Click
        Dim TXArray(2047) As Byte
        Dim chkSum As Byte = 0

        SyncLock COMPort

            TxDataLen = 0                   ' Total frame length - 3 bytes.ID + Len (=0) + 1 CHKSUM
            TXArray(0) = MotorID + 128      ' Header is  Motor ID with MSB set to indicate a control sequense
            TXArray(1) = 0                  ' 0x00 to get the controller config

            If COMPort.IsOpen Then

                chkSum = 0
                'calculate the check sum over the whole frame
                chkSum = chkSum Xor TXArray(0)
                chkSum = chkSum Xor TXArray(1)

                TXArray(2) = chkSum         ' store the check sum in the transmit buffer

                COMPort.DiscardInBuffer()

                If LIN.Checked = True Then
                    RxDataLen = TxDataLen + 3 + 56     'LIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                Else
                    RxDataLen = 56                     'RS232!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                End If


                COMPort.Write(TXArray, 0, 3)        ' Transmit the frame
            Else
                MsgBox("No COM Port Selected")
            End If

        End SyncLock
    End Sub
End Class
