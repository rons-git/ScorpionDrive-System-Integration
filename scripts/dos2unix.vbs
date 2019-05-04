Set oArgs = WScript.Arguments
If oArgs.Count <> 1 Then
	WScript.Echo  "Usage: {wscript | cscript} "& Wscript.ScriptName &" fileName " & vbcrlf
Else
	Dos2UnixFile(oArgs(0))
End If

Function Dos2UnixFile(filePath) 
	Dim Fso
	Set Fso = wscript.CreateObject("Scripting.FileSystemObject")
	Const ForReading = 1
    Const ForWriting = 2
    Const isCreateNew = True
	Set f=fso.OpenTextFile(filePath, ForReading, isCreateNew)
	s=replace(f.ReadAll,vbCrLf,vbLf) 'replace dos to unix format: vbcrlf=chr(13)chr(10)
	f.Close
	Set f=fso.OpenTextFile(filePath,ForWriting,isCreateNew)
	f.Write s
	f.Close
	Set f=Nothing
	Set Fso=Nothing
End Function