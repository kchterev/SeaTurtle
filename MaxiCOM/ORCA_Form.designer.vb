<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class ORCATester
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        If disposing AndAlso components IsNot Nothing Then
            components.Dispose()
        End If
        MyBase.Dispose(disposing)
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Me.Transmitted = New System.Windows.Forms.TextBox()
        Me.Received = New System.Windows.Forms.TextBox()
        Me.SendButton = New System.Windows.Forms.Button()
        Me.TransmittedLabel = New System.Windows.Forms.Label()
        Me.ReceivedLabel = New System.Windows.Forms.Label()
        Me.BaudRateBox = New System.Windows.Forms.ComboBox()
        Me.BaudRateLabel = New System.Windows.Forms.Label()
        Me.COMPortsBox = New System.Windows.Forms.ComboBox()
        Me.ComPortLabel = New System.Windows.Forms.Label()
        Me.ClearButton = New System.Windows.Forms.Button()
        Me.BreakButton = New System.Windows.Forms.Button()
        Me.Button1 = New System.Windows.Forms.Button()
        Me.Motor1SP = New System.Windows.Forms.TrackBar()
        Me.Motor2SP = New System.Windows.Forms.TrackBar()
        Me.Motor3SP = New System.Windows.Forms.TrackBar()
        Me.Motor4SP = New System.Windows.Forms.TrackBar()
        Me.SP1 = New System.Windows.Forms.TextBox()
        Me.SP2 = New System.Windows.Forms.TextBox()
        Me.SP3 = New System.Windows.Forms.TextBox()
        Me.SP4 = New System.Windows.Forms.TextBox()
        Me.Timer1 = New System.Windows.Forms.Timer(Me.components)
        Me.SendConfigButton = New System.Windows.Forms.Button()
        Me.ID_Box = New System.Windows.Forms.TextBox()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.Current1 = New System.Windows.Forms.TextBox()
        Me.Temp1 = New System.Windows.Forms.TextBox()
        Me.Temperature = New System.Windows.Forms.Label()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.RPM1 = New System.Windows.Forms.TextBox()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.Timer2 = New System.Windows.Forms.Timer(Me.components)
        Me.Timer3 = New System.Windows.Forms.Timer(Me.components)
        Me.Timer4 = New System.Windows.Forms.Timer(Me.components)
        Me.StartTimer = New System.Windows.Forms.Button()
        Me.StopTimer = New System.Windows.Forms.Button()
        Me.NextID = New System.Windows.Forms.Button()
        Me.RPM2 = New System.Windows.Forms.TextBox()
        Me.Temp2 = New System.Windows.Forms.TextBox()
        Me.Current2 = New System.Windows.Forms.TextBox()
        Me.RPM3 = New System.Windows.Forms.TextBox()
        Me.Temp3 = New System.Windows.Forms.TextBox()
        Me.Current3 = New System.Windows.Forms.TextBox()
        Me.GroupBox1 = New System.Windows.Forms.GroupBox()
        Me.GetConfigButton = New System.Windows.Forms.Button()
        Me.EditConfgiButton = New System.Windows.Forms.Button()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.GroupBox2 = New System.Windows.Forms.GroupBox()
        Me.Label7 = New System.Windows.Forms.Label()
        Me.Label6 = New System.Windows.Forms.Label()
        Me.Y = New System.Windows.Forms.TextBox()
        Me.X = New System.Windows.Forms.TextBox()
        Me.Label5 = New System.Windows.Forms.Label()
        Me.Compass = New System.Windows.Forms.TextBox()
        Me.GetStatusButton = New System.Windows.Forms.Button()
        Me.Label11 = New System.Windows.Forms.Label()
        Me.ExitButton = New System.Windows.Forms.Button()
        Me.LIN = New System.Windows.Forms.CheckBox()
        Me.TextBox1 = New System.Windows.Forms.TextBox()
        CType(Me.Motor1SP, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.Motor2SP, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.Motor3SP, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.Motor4SP, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.GroupBox1.SuspendLayout()
        Me.GroupBox2.SuspendLayout()
        Me.SuspendLayout()
        '
        'Transmitted
        '
        Me.Transmitted.Font = New System.Drawing.Font("Courier New", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Transmitted.Location = New System.Drawing.Point(12, 334)
        Me.Transmitted.Multiline = True
        Me.Transmitted.Name = "Transmitted"
        Me.Transmitted.Size = New System.Drawing.Size(914, 53)
        Me.Transmitted.TabIndex = 0
        '
        'Received
        '
        Me.Received.Font = New System.Drawing.Font("Courier New", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Received.Location = New System.Drawing.Point(12, 68)
        Me.Received.Multiline = True
        Me.Received.Name = "Received"
        Me.Received.ScrollBars = System.Windows.Forms.ScrollBars.Vertical
        Me.Received.Size = New System.Drawing.Size(914, 214)
        Me.Received.TabIndex = 1
        '
        'SendButton
        '
        Me.SendButton.Location = New System.Drawing.Point(86, 300)
        Me.SendButton.Name = "SendButton"
        Me.SendButton.Size = New System.Drawing.Size(75, 23)
        Me.SendButton.TabIndex = 2
        Me.SendButton.Text = "Send Data"
        Me.SendButton.UseVisualStyleBackColor = True
        '
        'TransmittedLabel
        '
        Me.TransmittedLabel.AutoSize = True
        Me.TransmittedLabel.Location = New System.Drawing.Point(21, 315)
        Me.TransmittedLabel.Name = "TransmittedLabel"
        Me.TransmittedLabel.Size = New System.Drawing.Size(47, 13)
        Me.TransmittedLabel.TabIndex = 3
        Me.TransmittedLabel.Text = "Transmit"
        '
        'ReceivedLabel
        '
        Me.ReceivedLabel.AutoSize = True
        Me.ReceivedLabel.Location = New System.Drawing.Point(21, 52)
        Me.ReceivedLabel.Name = "ReceivedLabel"
        Me.ReceivedLabel.Size = New System.Drawing.Size(53, 13)
        Me.ReceivedLabel.TabIndex = 4
        Me.ReceivedLabel.Text = "Received"
        '
        'BaudRateBox
        '
        Me.BaudRateBox.FormattingEnabled = True
        Me.BaudRateBox.Location = New System.Drawing.Point(240, 17)
        Me.BaudRateBox.Name = "BaudRateBox"
        Me.BaudRateBox.Size = New System.Drawing.Size(83, 21)
        Me.BaudRateBox.TabIndex = 5
        '
        'BaudRateLabel
        '
        Me.BaudRateLabel.AutoSize = True
        Me.BaudRateLabel.Location = New System.Drawing.Point(159, 20)
        Me.BaudRateLabel.Name = "BaudRateLabel"
        Me.BaudRateLabel.Size = New System.Drawing.Size(75, 13)
        Me.BaudRateLabel.TabIndex = 6
        Me.BaudRateLabel.Text = "Bit Rate (bit/s)"
        '
        'COMPortsBox
        '
        Me.COMPortsBox.FormattingEnabled = True
        Me.COMPortsBox.Location = New System.Drawing.Point(80, 17)
        Me.COMPortsBox.Name = "COMPortsBox"
        Me.COMPortsBox.Size = New System.Drawing.Size(73, 21)
        Me.COMPortsBox.TabIndex = 7
        '
        'ComPortLabel
        '
        Me.ComPortLabel.AutoSize = True
        Me.ComPortLabel.Location = New System.Drawing.Point(21, 20)
        Me.ComPortLabel.Name = "ComPortLabel"
        Me.ComPortLabel.Size = New System.Drawing.Size(53, 13)
        Me.ComPortLabel.TabIndex = 8
        Me.ComPortLabel.Text = "COM Port"
        '
        'ClearButton
        '
        Me.ClearButton.Location = New System.Drawing.Point(381, 300)
        Me.ClearButton.Name = "ClearButton"
        Me.ClearButton.Size = New System.Drawing.Size(75, 23)
        Me.ClearButton.TabIndex = 9
        Me.ClearButton.Text = "Clear"
        Me.ClearButton.UseVisualStyleBackColor = True
        '
        'BreakButton
        '
        Me.BreakButton.Enabled = False
        Me.BreakButton.Location = New System.Drawing.Point(10, 19)
        Me.BreakButton.Name = "BreakButton"
        Me.BreakButton.Size = New System.Drawing.Size(71, 23)
        Me.BreakButton.TabIndex = 10
        Me.BreakButton.Text = "Reset"
        Me.BreakButton.UseVisualStyleBackColor = True
        '
        'Button1
        '
        Me.Button1.Location = New System.Drawing.Point(479, 300)
        Me.Button1.Name = "Button1"
        Me.Button1.Size = New System.Drawing.Size(75, 23)
        Me.Button1.TabIndex = 21
        Me.Button1.Text = "Save"
        Me.Button1.UseVisualStyleBackColor = True
        '
        'Motor1SP
        '
        Me.Motor1SP.LargeChange = 100
        Me.Motor1SP.Location = New System.Drawing.Point(118, 112)
        Me.Motor1SP.Maximum = 10000
        Me.Motor1SP.Minimum = -10000
        Me.Motor1SP.Name = "Motor1SP"
        Me.Motor1SP.Size = New System.Drawing.Size(150, 45)
        Me.Motor1SP.SmallChange = 10
        Me.Motor1SP.TabIndex = 22
        '
        'Motor2SP
        '
        Me.Motor2SP.LargeChange = 100
        Me.Motor2SP.Location = New System.Drawing.Point(304, 112)
        Me.Motor2SP.Maximum = 10000
        Me.Motor2SP.Minimum = -10000
        Me.Motor2SP.Name = "Motor2SP"
        Me.Motor2SP.Size = New System.Drawing.Size(150, 45)
        Me.Motor2SP.SmallChange = 10
        Me.Motor2SP.TabIndex = 23
        '
        'Motor3SP
        '
        Me.Motor3SP.LargeChange = 100
        Me.Motor3SP.Location = New System.Drawing.Point(487, 112)
        Me.Motor3SP.Maximum = 10000
        Me.Motor3SP.Minimum = -10000
        Me.Motor3SP.Name = "Motor3SP"
        Me.Motor3SP.Size = New System.Drawing.Size(150, 45)
        Me.Motor3SP.SmallChange = 10
        Me.Motor3SP.TabIndex = 24
        '
        'Motor4SP
        '
        Me.Motor4SP.LargeChange = 100
        Me.Motor4SP.Location = New System.Drawing.Point(690, 112)
        Me.Motor4SP.Maximum = 32767
        Me.Motor4SP.Minimum = -32768
        Me.Motor4SP.Name = "Motor4SP"
        Me.Motor4SP.Size = New System.Drawing.Size(150, 45)
        Me.Motor4SP.SmallChange = 10
        Me.Motor4SP.TabIndex = 25
        '
        'SP1
        '
        Me.SP1.AcceptsReturn = True
        Me.SP1.Location = New System.Drawing.Point(147, 149)
        Me.SP1.Name = "SP1"
        Me.SP1.Size = New System.Drawing.Size(100, 20)
        Me.SP1.TabIndex = 33
        Me.SP1.Text = "0"
        '
        'SP2
        '
        Me.SP2.AcceptsReturn = True
        Me.SP2.Location = New System.Drawing.Point(327, 149)
        Me.SP2.Name = "SP2"
        Me.SP2.Size = New System.Drawing.Size(100, 20)
        Me.SP2.TabIndex = 34
        Me.SP2.Text = "0"
        '
        'SP3
        '
        Me.SP3.AcceptsReturn = True
        Me.SP3.Location = New System.Drawing.Point(516, 149)
        Me.SP3.Name = "SP3"
        Me.SP3.Size = New System.Drawing.Size(100, 20)
        Me.SP3.TabIndex = 35
        Me.SP3.Text = "0"
        '
        'SP4
        '
        Me.SP4.AcceptsReturn = True
        Me.SP4.Location = New System.Drawing.Point(714, 149)
        Me.SP4.Name = "SP4"
        Me.SP4.Size = New System.Drawing.Size(98, 20)
        Me.SP4.TabIndex = 36
        Me.SP4.Text = "0"
        '
        'Timer1
        '
        '
        'SendConfigButton
        '
        Me.SendConfigButton.Enabled = False
        Me.SendConfigButton.Location = New System.Drawing.Point(487, 19)
        Me.SendConfigButton.Name = "SendConfigButton"
        Me.SendConfigButton.Size = New System.Drawing.Size(75, 23)
        Me.SendConfigButton.TabIndex = 38
        Me.SendConfigButton.Text = "Send Config"
        Me.SendConfigButton.UseVisualStyleBackColor = True
        '
        'ID_Box
        '
        Me.ID_Box.AcceptsReturn = True
        Me.ID_Box.Location = New System.Drawing.Point(367, 22)
        Me.ID_Box.Name = "ID_Box"
        Me.ID_Box.Size = New System.Drawing.Size(100, 20)
        Me.ID_Box.TabIndex = 49
        Me.ID_Box.Text = "0"
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Location = New System.Drawing.Point(343, 29)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(18, 13)
        Me.Label8.TabIndex = 51
        Me.Label8.Text = "ID"
        '
        'Current1
        '
        Me.Current1.AcceptsReturn = True
        Me.Current1.Location = New System.Drawing.Point(147, 61)
        Me.Current1.Name = "Current1"
        Me.Current1.Size = New System.Drawing.Size(100, 20)
        Me.Current1.TabIndex = 52
        Me.Current1.Text = "0"
        '
        'Temp1
        '
        Me.Temp1.AcceptsReturn = True
        Me.Temp1.Location = New System.Drawing.Point(147, 87)
        Me.Temp1.Name = "Temp1"
        Me.Temp1.Size = New System.Drawing.Size(100, 20)
        Me.Temp1.TabIndex = 53
        Me.Temp1.Text = "0"
        '
        'Temperature
        '
        Me.Temperature.AutoSize = True
        Me.Temperature.Location = New System.Drawing.Point(109, 90)
        Me.Temperature.Name = "Temperature"
        Me.Temperature.Size = New System.Drawing.Size(34, 13)
        Me.Temperature.TabIndex = 54
        Me.Temperature.Text = "Temp"
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Location = New System.Drawing.Point(106, 64)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(41, 13)
        Me.Label10.TabIndex = 55
        Me.Label10.Text = "Current"
        '
        'RPM1
        '
        Me.RPM1.AcceptsReturn = True
        Me.RPM1.Location = New System.Drawing.Point(147, 35)
        Me.RPM1.Name = "RPM1"
        Me.RPM1.Size = New System.Drawing.Size(100, 20)
        Me.RPM1.TabIndex = 56
        Me.RPM1.Text = "0"
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Location = New System.Drawing.Point(87, 38)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(60, 13)
        Me.Label9.TabIndex = 57
        Me.Label9.Text = "Read RPM"
        '
        'StartTimer
        '
        Me.StartTimer.Enabled = False
        Me.StartTimer.Location = New System.Drawing.Point(10, 19)
        Me.StartTimer.Name = "StartTimer"
        Me.StartTimer.Size = New System.Drawing.Size(71, 24)
        Me.StartTimer.TabIndex = 58
        Me.StartTimer.Text = "Start"
        Me.StartTimer.UseVisualStyleBackColor = True
        '
        'StopTimer
        '
        Me.StopTimer.Enabled = False
        Me.StopTimer.Location = New System.Drawing.Point(10, 52)
        Me.StopTimer.Name = "StopTimer"
        Me.StopTimer.Size = New System.Drawing.Size(71, 25)
        Me.StopTimer.TabIndex = 59
        Me.StopTimer.Text = "Stop"
        Me.StopTimer.UseVisualStyleBackColor = True
        '
        'NextID
        '
        Me.NextID.Enabled = False
        Me.NextID.Location = New System.Drawing.Point(10, 90)
        Me.NextID.Name = "NextID"
        Me.NextID.Size = New System.Drawing.Size(71, 24)
        Me.NextID.TabIndex = 60
        Me.NextID.Text = "Next"
        Me.NextID.UseVisualStyleBackColor = True
        '
        'RPM2
        '
        Me.RPM2.AcceptsReturn = True
        Me.RPM2.Location = New System.Drawing.Point(327, 35)
        Me.RPM2.Name = "RPM2"
        Me.RPM2.Size = New System.Drawing.Size(100, 20)
        Me.RPM2.TabIndex = 63
        Me.RPM2.Text = "0"
        '
        'Temp2
        '
        Me.Temp2.AcceptsReturn = True
        Me.Temp2.Location = New System.Drawing.Point(327, 87)
        Me.Temp2.Name = "Temp2"
        Me.Temp2.Size = New System.Drawing.Size(100, 20)
        Me.Temp2.TabIndex = 62
        Me.Temp2.Text = "0"
        '
        'Current2
        '
        Me.Current2.AcceptsReturn = True
        Me.Current2.Location = New System.Drawing.Point(327, 61)
        Me.Current2.Name = "Current2"
        Me.Current2.Size = New System.Drawing.Size(100, 20)
        Me.Current2.TabIndex = 61
        Me.Current2.Text = "0"
        '
        'RPM3
        '
        Me.RPM3.AcceptsReturn = True
        Me.RPM3.Location = New System.Drawing.Point(516, 31)
        Me.RPM3.Name = "RPM3"
        Me.RPM3.Size = New System.Drawing.Size(100, 20)
        Me.RPM3.TabIndex = 66
        Me.RPM3.Text = "0"
        '
        'Temp3
        '
        Me.Temp3.AcceptsReturn = True
        Me.Temp3.Location = New System.Drawing.Point(516, 83)
        Me.Temp3.Name = "Temp3"
        Me.Temp3.Size = New System.Drawing.Size(100, 20)
        Me.Temp3.TabIndex = 65
        Me.Temp3.Text = "0"
        '
        'Current3
        '
        Me.Current3.AcceptsReturn = True
        Me.Current3.Location = New System.Drawing.Point(516, 57)
        Me.Current3.Name = "Current3"
        Me.Current3.Size = New System.Drawing.Size(100, 20)
        Me.Current3.TabIndex = 64
        Me.Current3.Text = "0"
        '
        'GroupBox1
        '
        Me.GroupBox1.Controls.Add(Me.GetConfigButton)
        Me.GroupBox1.Controls.Add(Me.EditConfgiButton)
        Me.GroupBox1.Controls.Add(Me.Label8)
        Me.GroupBox1.Controls.Add(Me.ID_Box)
        Me.GroupBox1.Controls.Add(Me.SendConfigButton)
        Me.GroupBox1.Controls.Add(Me.BreakButton)
        Me.GroupBox1.Location = New System.Drawing.Point(14, 575)
        Me.GroupBox1.Name = "GroupBox1"
        Me.GroupBox1.Size = New System.Drawing.Size(637, 55)
        Me.GroupBox1.TabIndex = 70
        Me.GroupBox1.TabStop = False
        Me.GroupBox1.Text = "Admin"
        '
        'GetConfigButton
        '
        Me.GetConfigButton.Enabled = False
        Me.GetConfigButton.Location = New System.Drawing.Point(147, 19)
        Me.GetConfigButton.Name = "GetConfigButton"
        Me.GetConfigButton.Size = New System.Drawing.Size(71, 24)
        Me.GetConfigButton.TabIndex = 77
        Me.GetConfigButton.Text = "Get Config"
        Me.GetConfigButton.UseVisualStyleBackColor = True
        '
        'EditConfgiButton
        '
        Me.EditConfgiButton.Location = New System.Drawing.Point(243, 19)
        Me.EditConfgiButton.Name = "EditConfgiButton"
        Me.EditConfgiButton.Size = New System.Drawing.Size(75, 23)
        Me.EditConfgiButton.TabIndex = 52
        Me.EditConfgiButton.Text = "Edit Config"
        Me.EditConfgiButton.UseVisualStyleBackColor = True
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(170, 16)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(55, 13)
        Me.Label1.TabIndex = 71
        Me.Label1.Text = "Thruster 1"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(348, 16)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(55, 13)
        Me.Label2.TabIndex = 72
        Me.Label2.Text = "Thruster 2"
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Location = New System.Drawing.Point(542, 13)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(55, 13)
        Me.Label3.TabIndex = 73
        Me.Label3.Text = "Thruster 3"
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.Location = New System.Drawing.Point(838, 131)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(60, 13)
        Me.Label4.TabIndex = 74
        Me.Label4.Text = "Camera Tilt"
        '
        'GroupBox2
        '
        Me.GroupBox2.Controls.Add(Me.Label7)
        Me.GroupBox2.Controls.Add(Me.Label6)
        Me.GroupBox2.Controls.Add(Me.Y)
        Me.GroupBox2.Controls.Add(Me.X)
        Me.GroupBox2.Controls.Add(Me.Label5)
        Me.GroupBox2.Controls.Add(Me.Compass)
        Me.GroupBox2.Controls.Add(Me.GetStatusButton)
        Me.GroupBox2.Controls.Add(Me.Label11)
        Me.GroupBox2.Controls.Add(Me.Label4)
        Me.GroupBox2.Controls.Add(Me.Label3)
        Me.GroupBox2.Controls.Add(Me.Label2)
        Me.GroupBox2.Controls.Add(Me.Label1)
        Me.GroupBox2.Controls.Add(Me.RPM3)
        Me.GroupBox2.Controls.Add(Me.Temp3)
        Me.GroupBox2.Controls.Add(Me.Current3)
        Me.GroupBox2.Controls.Add(Me.RPM2)
        Me.GroupBox2.Controls.Add(Me.Temp2)
        Me.GroupBox2.Controls.Add(Me.Current2)
        Me.GroupBox2.Controls.Add(Me.NextID)
        Me.GroupBox2.Controls.Add(Me.StopTimer)
        Me.GroupBox2.Controls.Add(Me.StartTimer)
        Me.GroupBox2.Controls.Add(Me.Label9)
        Me.GroupBox2.Controls.Add(Me.RPM1)
        Me.GroupBox2.Controls.Add(Me.Label10)
        Me.GroupBox2.Controls.Add(Me.Temperature)
        Me.GroupBox2.Controls.Add(Me.Temp1)
        Me.GroupBox2.Controls.Add(Me.Current1)
        Me.GroupBox2.Controls.Add(Me.SP4)
        Me.GroupBox2.Controls.Add(Me.SP3)
        Me.GroupBox2.Controls.Add(Me.SP2)
        Me.GroupBox2.Controls.Add(Me.SP1)
        Me.GroupBox2.Controls.Add(Me.Motor4SP)
        Me.GroupBox2.Controls.Add(Me.Motor3SP)
        Me.GroupBox2.Controls.Add(Me.Motor2SP)
        Me.GroupBox2.Controls.Add(Me.Motor1SP)
        Me.GroupBox2.Location = New System.Drawing.Point(14, 393)
        Me.GroupBox2.Name = "GroupBox2"
        Me.GroupBox2.Size = New System.Drawing.Size(910, 182)
        Me.GroupBox2.TabIndex = 75
        Me.GroupBox2.TabStop = False
        Me.GroupBox2.Text = "control"
        '
        'Label7
        '
        Me.Label7.AutoSize = True
        Me.Label7.Location = New System.Drawing.Point(820, 81)
        Me.Label7.Name = "Label7"
        Me.Label7.Size = New System.Drawing.Size(57, 13)
        Me.Label7.TabIndex = 82
        Me.Label7.Text = "pitch (deg)"
        '
        'Label6
        '
        Me.Label6.AutoSize = True
        Me.Label6.Location = New System.Drawing.Point(820, 57)
        Me.Label6.Name = "Label6"
        Me.Label6.Size = New System.Drawing.Size(47, 13)
        Me.Label6.TabIndex = 81
        Me.Label6.Text = "roll (deg)"
        '
        'Y
        '
        Me.Y.AcceptsReturn = True
        Me.Y.Location = New System.Drawing.Point(714, 81)
        Me.Y.Name = "Y"
        Me.Y.Size = New System.Drawing.Size(100, 20)
        Me.Y.TabIndex = 80
        Me.Y.Text = "0"
        '
        'X
        '
        Me.X.AcceptsReturn = True
        Me.X.Location = New System.Drawing.Point(714, 55)
        Me.X.Name = "X"
        Me.X.Size = New System.Drawing.Size(100, 20)
        Me.X.TabIndex = 79
        Me.X.Text = "0"
        '
        'Label5
        '
        Me.Label5.AutoSize = True
        Me.Label5.Location = New System.Drawing.Point(820, 34)
        Me.Label5.Name = "Label5"
        Me.Label5.Size = New System.Drawing.Size(74, 13)
        Me.Label5.TabIndex = 78
        Me.Label5.Text = "Heading (deg)"
        '
        'Compass
        '
        Me.Compass.AcceptsReturn = True
        Me.Compass.Location = New System.Drawing.Point(714, 31)
        Me.Compass.Name = "Compass"
        Me.Compass.Size = New System.Drawing.Size(100, 20)
        Me.Compass.TabIndex = 77
        Me.Compass.Text = "0"
        '
        'GetStatusButton
        '
        Me.GetStatusButton.Enabled = False
        Me.GetStatusButton.Location = New System.Drawing.Point(10, 145)
        Me.GetStatusButton.Name = "GetStatusButton"
        Me.GetStatusButton.Size = New System.Drawing.Size(71, 24)
        Me.GetStatusButton.TabIndex = 76
        Me.GetStatusButton.Text = "Get Status"
        Me.GetStatusButton.UseVisualStyleBackColor = True
        '
        'Label11
        '
        Me.Label11.AutoSize = True
        Me.Label11.Location = New System.Drawing.Point(99, 152)
        Me.Label11.Name = "Label11"
        Me.Label11.Size = New System.Drawing.Size(31, 13)
        Me.Label11.TabIndex = 75
        Me.Label11.Text = "RPM"
        '
        'ExitButton
        '
        Me.ExitButton.Location = New System.Drawing.Point(855, 594)
        Me.ExitButton.Name = "ExitButton"
        Me.ExitButton.Size = New System.Drawing.Size(71, 23)
        Me.ExitButton.TabIndex = 78
        Me.ExitButton.Text = "Exit"
        Me.ExitButton.UseVisualStyleBackColor = True
        '
        'LIN
        '
        Me.LIN.AutoSize = True
        Me.LIN.Checked = True
        Me.LIN.CheckState = System.Windows.Forms.CheckState.Checked
        Me.LIN.Location = New System.Drawing.Point(381, 20)
        Me.LIN.Name = "LIN"
        Me.LIN.Size = New System.Drawing.Size(86, 17)
        Me.LIN.TabIndex = 79
        Me.LIN.Text = "LIN Echo on"
        Me.LIN.UseVisualStyleBackColor = True
        '
        'TextBox1
        '
        Me.TextBox1.AcceptsReturn = True
        Me.TextBox1.Location = New System.Drawing.Point(657, 594)
        Me.TextBox1.Name = "TextBox1"
        Me.TextBox1.Size = New System.Drawing.Size(169, 20)
        Me.TextBox1.TabIndex = 80
        Me.TextBox1.Text = "0"
        '
        'ORCATester
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(946, 643)
        Me.Controls.Add(Me.TextBox1)
        Me.Controls.Add(Me.LIN)
        Me.Controls.Add(Me.ExitButton)
        Me.Controls.Add(Me.GroupBox2)
        Me.Controls.Add(Me.GroupBox1)
        Me.Controls.Add(Me.Button1)
        Me.Controls.Add(Me.ClearButton)
        Me.Controls.Add(Me.ComPortLabel)
        Me.Controls.Add(Me.COMPortsBox)
        Me.Controls.Add(Me.BaudRateLabel)
        Me.Controls.Add(Me.BaudRateBox)
        Me.Controls.Add(Me.ReceivedLabel)
        Me.Controls.Add(Me.TransmittedLabel)
        Me.Controls.Add(Me.SendButton)
        Me.Controls.Add(Me.Received)
        Me.Controls.Add(Me.Transmitted)
        Me.Name = "ORCATester"
        Me.Text = "O.R.C.A. Tester"
        CType(Me.Motor1SP, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.Motor2SP, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.Motor3SP, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.Motor4SP, System.ComponentModel.ISupportInitialize).EndInit()
        Me.GroupBox1.ResumeLayout(False)
        Me.GroupBox1.PerformLayout()
        Me.GroupBox2.ResumeLayout(False)
        Me.GroupBox2.PerformLayout()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents Transmitted As System.Windows.Forms.TextBox
    Friend WithEvents Received As System.Windows.Forms.TextBox
    Friend WithEvents SendButton As System.Windows.Forms.Button
    Friend WithEvents TransmittedLabel As System.Windows.Forms.Label
    Friend WithEvents ReceivedLabel As System.Windows.Forms.Label
    Friend WithEvents BaudRateBox As System.Windows.Forms.ComboBox
    Friend WithEvents BaudRateLabel As System.Windows.Forms.Label
    Friend WithEvents COMPortsBox As System.Windows.Forms.ComboBox
    Friend WithEvents ComPortLabel As System.Windows.Forms.Label
    Friend WithEvents ClearButton As System.Windows.Forms.Button
    Friend WithEvents BreakButton As System.Windows.Forms.Button

    Public Sub New()

        ' This call is required by the Windows Form Designer.
        InitializeComponent()

        ' Add any initialization after the InitializeComponent() call.

    End Sub
    Friend WithEvents Button1 As System.Windows.Forms.Button
    Friend WithEvents Motor1SP As System.Windows.Forms.TrackBar
    Friend WithEvents Motor2SP As System.Windows.Forms.TrackBar
    Friend WithEvents Motor3SP As System.Windows.Forms.TrackBar
    Friend WithEvents Motor4SP As System.Windows.Forms.TrackBar
    Friend WithEvents SP1 As System.Windows.Forms.TextBox
    Friend WithEvents SP2 As System.Windows.Forms.TextBox
    Friend WithEvents SP3 As System.Windows.Forms.TextBox
    Friend WithEvents SP4 As System.Windows.Forms.TextBox
    Friend WithEvents Timer1 As System.Windows.Forms.Timer
    Friend WithEvents SendConfigButton As System.Windows.Forms.Button
    Friend WithEvents ID_Box As System.Windows.Forms.TextBox
    Friend WithEvents Label8 As System.Windows.Forms.Label
    Friend WithEvents Current1 As System.Windows.Forms.TextBox
    Friend WithEvents Temp1 As System.Windows.Forms.TextBox
    Friend WithEvents Temperature As System.Windows.Forms.Label
    Friend WithEvents Label10 As System.Windows.Forms.Label
    Friend WithEvents RPM1 As System.Windows.Forms.TextBox
    Friend WithEvents Label9 As System.Windows.Forms.Label
    Friend WithEvents Timer2 As System.Windows.Forms.Timer
    Friend WithEvents Timer3 As System.Windows.Forms.Timer
    Friend WithEvents Timer4 As System.Windows.Forms.Timer
    Friend WithEvents StartTimer As System.Windows.Forms.Button
    Friend WithEvents StopTimer As System.Windows.Forms.Button
    Friend WithEvents NextID As System.Windows.Forms.Button
    Friend WithEvents RPM2 As System.Windows.Forms.TextBox
    Friend WithEvents Temp2 As System.Windows.Forms.TextBox
    Friend WithEvents Current2 As System.Windows.Forms.TextBox
    Friend WithEvents RPM3 As System.Windows.Forms.TextBox
    Friend WithEvents Temp3 As System.Windows.Forms.TextBox
    Friend WithEvents Current3 As System.Windows.Forms.TextBox
    Friend WithEvents GroupBox1 As System.Windows.Forms.GroupBox
    Friend WithEvents Label1 As System.Windows.Forms.Label
    Friend WithEvents Label2 As System.Windows.Forms.Label
    Friend WithEvents Label3 As System.Windows.Forms.Label
    Friend WithEvents Label4 As System.Windows.Forms.Label
    Friend WithEvents GroupBox2 As System.Windows.Forms.GroupBox
    Friend WithEvents Label11 As System.Windows.Forms.Label
    Friend WithEvents EditConfgiButton As System.Windows.Forms.Button
    Friend WithEvents GetStatusButton As System.Windows.Forms.Button
    Friend WithEvents GetConfigButton As System.Windows.Forms.Button
    Friend WithEvents ExitButton As System.Windows.Forms.Button
    Friend WithEvents Label5 As Label
    Friend WithEvents Compass As TextBox
    Friend WithEvents Label7 As Label
    Friend WithEvents Label6 As Label
    Friend WithEvents Y As TextBox
    Friend WithEvents X As TextBox
    Friend WithEvents LIN As CheckBox
    Friend WithEvents TextBox1 As TextBox
End Class
