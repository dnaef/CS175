from tkinter import *
import serial
import threading

port = "COM4"
baud = 38400
sent = 0
count = 0
ser = serial.Serial(port, baud, timeout=1)
ser.setDTR(1)
 
#while True:
#    cmd = input("Enter command or 'exit':")
#        # for Python 3
#    if (cmd == 'exit'):
#        ser.close()
#        exit()
##    else:
#        sent = ser.write(cmd.encode('utf-8')) 
 #       count = count + sent
 #       print (count)

        
#KEYDOWN click function
 
def click():
    
    entered_text=textentry.get()
    sent = ser.write(entered_text.encode('ascii', 'ignore'))
    for i in range(0,2):
        textentry.delete(0, 'end')
        poll_serial_port()
        i+=1
def enter(event):
    entered_text=textentry.get()
    sent = ser.write(entered_text.encode('ascii', 'ignore'))
    textentry.delete(0, 'end')
    poll_serial_port()
    
def exit_program():
    ser.close()
    exit()
def Start():
    cmd = "R"
    ser.write(cmd.encode('ascii', 'ignore'))
    
def Stop():
    cmd = "S"
    ser.write(cmd.encode('ascii', 'ignore'))
def poll_serial_port():
    data =""
    if (ser.inWaiting()>0):
        data = ser.read(8);
        status.configure(text=data)
        root.after(1, poll_serial_port())

root = Tk()
#set the parameters for the root/window
root.bind('<Return>', enter)

root.title("DJ Smart Conveyor GUI") #title of the window
root.geometry("400x200") #size of the array

#background
root.configure(background="black")

#label

status = Label (root, text = "text", bg="black", fg="white", font="none 12 bold")
status.grid(row=1,column=0,sticky=W)

#update button
Button(root, text = "Update", width = 6, command = click).grid(row=2, column=1,sticky=W)

#exit button
Button(root, text = "Exit", width = 6, command = exit_program).grid(row=4, column=0,sticky=W)

#Start button
Button(root, text = "Start", width = 6, command = Start).grid(row=5, column=0,sticky=W)
#Start button
Button(root, text = "Stop", width = 6, command = Stop).grid(row=5, column=1,sticky=W)
#Speed
textentry = Entry(root, width=20, bg="white")
textentry.insert(END, 'Send Character')
textentry.grid(row=2, column=0,sticky=W)

poll_serial_port()
root.mainloop()
    


