
Imports System.ComponentModel
Imports System.Runtime.InteropServices

Public Class Joystick
    Inherits NativeWindow

    Private parent As Form
    Private Const MM_JOY1MOVE As Integer = &H3A0

    ' Public Event Move(ByVal joystickPosition As Point)
    Public btnValue As String
    Public p As JoyPosition
    Public Event Up()
    Public Event Down()
    Public Event Left()
    Public Event Right()
    Public Event Joy()

    <StructLayout(LayoutKind.Explicit)> _
    Public Structure JoyPosition
        <FieldOffset(0)> _
        Public Raw As IntPtr
        <FieldOffset(0)> _
        Public XPos As UShort
        <FieldOffset(2)> _
        Public YPos As UShort
    End Structure

    Private Class NativeMethods

        Private Sub New()
        End Sub

        ' This is a "Stub" function - it has no code in its body.
        ' There is a similarly named function inside a dll that comes with windows called
        ' winmm.dll.
        ' The .Net framework will route calls to this function, through to the dll file.
        <DllImport("winmm", CallingConvention:=CallingConvention.Winapi, EntryPoint:="joySetCapture", SetLastError:=True)> _
        Public Shared Function JoySetCapture(ByVal hwnd As IntPtr, ByVal uJoyID As Integer, ByVal uPeriod As Integer, <MarshalAs(UnmanagedType.Bool)> ByVal changed As Boolean) As Integer
        End Function

    End Class

    Public Sub New(ByVal parent As Form, ByVal joyId As Integer)
        AddHandler parent.HandleCreated, AddressOf Me.OnHandleCreated
        AddHandler parent.HandleDestroyed, AddressOf Me.OnHandleDestroyed
        AssignHandle(parent.Handle)
        Me.parent = parent
        Dim result As Integer = NativeMethods.JoySetCapture(Me.Handle, joyId, 100, True)
    End Sub

    Private Sub OnHandleCreated(ByVal sender As Object, ByVal e As EventArgs)
        AssignHandle(DirectCast(sender, Form).Handle)
    End Sub

    Private Sub OnHandleDestroyed(ByVal sender As Object, ByVal e As EventArgs)
        ReleaseHandle()
    End Sub

    Protected Overrides Sub WndProc(ByRef m As System.Windows.Forms.Message)
        If m.Msg = MM_JOY1MOVE Then
            ' Joystick co-ords.
            ' (0,0)         (32768,0)           (65535, 0)
            '
            '
            '
            ' (0, 32768)    (32768, 32768)      (65535, 32768)
            '
            '
            '
            '
            ' (0, 65535)    (32768, 65535)      (65535, 65535)
            '
            p.Raw = m.LParam
            ' RaiseEvent Move(New Point(p.XPos, p.YPos))
            ' If p.XPos > 16384 AndAlso p.XPos < 49152 Then
            '' X is near the centre line.
            ' If p.YPos < 32768 - 50 Then
            '' Y is near the top.
            ' RaiseEvent Up()
            ' ElseIf p.YPos > 32768 + 50 Then
            '  ' Y is near the bottom.
            '   RaiseEvent Down()
            ' End If
            '  Else
            'If p.YPos > 16384 AndAlso p.YPos < 49152 Then
            '' Y is near the centre line
            ' If p.XPos < 32768 - 50 Then
            ' X is near the left.
            'RaiseEvent Left()
            'ElseIf p.XPos > 32768 + 50 Then
            ' X is near the right
            ' RaiseEvent Right()
            ' End If
            ' End If
            'End If

            RaiseEvent Joy()
        End If
        If btnValue <> m.WParam.ToString Then
            btnValue = m.WParam.ToString
        End If
        MyBase.WndProc(m)
    End Sub

End Class