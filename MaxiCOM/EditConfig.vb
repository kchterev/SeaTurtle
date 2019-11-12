Public Class EditConfig

    Private Sub EditConfig_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load
        RichTextBox1.Text = System.IO.File.ReadAllText("config.ini")
        My.Computer.FileSystem.WriteAllText("config.bak", RichTextBox1.Text, False)
    End Sub

    Private Sub CancelButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles CancelButton.Click
        Me.Close()
    End Sub

    Private Sub SaveButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles SaveButton.Click
        My.Computer.FileSystem.WriteAllText("config.ini", RichTextBox1.Text, False)
    End Sub
End Class