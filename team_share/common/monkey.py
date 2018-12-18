#!/usr/bin/env python
# -*- coding: utf-8 -*-

import wx
import os
import time
import string

class MyFrame(wx.Frame):
    delayDefault = "2"
    seedDefault = "5000000"
    executionFrequencyDefault = "60000000"
    logDir = "./"
    def __init__(self):

        wx.Frame.__init__(self, None, -1, "My Frame", size=(500, 800))
        panel = wx.Panel(self, -1)

        xPos = 10
        xPos1 = 180
        yPos  = 12
        yDelta = 40
        excuteMode = ["忽略程序崩溃",
                    "忽略程序无响应",
                    "忽略安全异常",
                    "出错中断程序",
                    "本地代码导致的崩溃",
                    "默认"
                    ]


        logMode = ["简单","普通","详细"]
        executionModeDefault = excuteMode[0]

        menuBar = wx.MenuBar()
        menu1 = wx.Menu("")
        menuBar.Append(menu1, "File")
        self.SetMenuBar(menuBar)

        wx.StaticText(panel, -1, "种子数:", pos=(xPos, yPos))
        self.seedCtrl = wx.TextCtrl(panel, -1, "", pos=(xPos1, yPos))
        self.seedCtrl.Bind(wx.EVT_KILL_FOCUS, self.OnAction)
        self.seedCtrl.SetFocus()

        wx.StaticText(panel, -1, "执行次数:", pos=(xPos, yPos+yDelta))
        self.excuteNumCtrl = wx.TextCtrl(panel, -1, "", pos=(xPos1, yPos+yDelta))

        wx.StaticText(panel, -1, "延时:", pos=(xPos, yPos+2*yDelta))
        self.delayNumCtrl = wx.TextCtrl(panel, -1, "", pos=(xPos1, yPos+2*yDelta))

        wx.StaticText(panel, -1, "执行方式:", pos=(xPos, yPos+3*yDelta))

        self.excuteModeCtrl = wx.ComboBox(panel, -1, "", (xPos1,yPos+3*yDelta), choices=excuteMode,style=wx.CB_DROPDOWN)

        self.checkListBox = wx.CheckListBox(panel, -1, (xPos, yPos+4*yDelta ), (400, 350), [])

        yPoslayout = yPos+14*yDelta

        wx.StaticText(panel, -1, "日志输出等级:", pos=(xPos, yPoslayout-yDelta))
        self.logModeCtrl = wx.ComboBox(panel, -1, "", (xPos1,yPoslayout-yDelta), choices=logMode,style=wx.CB_DROPDOWN)

        self.readButton = wx.Button(panel, -1, "读取程序包", pos=(xPos, yPoslayout))
        self.Bind(wx.EVT_BUTTON, self.OnReadClick, self.readButton)
        self.readButton.SetDefault()

        self.selectButton = wx.Button(panel, -1, "全部选择", pos=(xPos+120, yPoslayout))
        self.Bind(wx.EVT_BUTTON, self.OnSelectAllClick, self.selectButton)
        self.selectButton.SetDefault()

        self.unselectButton = wx.Button(panel, -1, "全部取消", pos=(xPos+120*2, yPoslayout))
        self.Bind(wx.EVT_BUTTON, self.OnUnselectClick, self.unselectButton)

        self.defaultButton = wx.Button(panel, -1, "默认参数", pos=(xPos, yPoslayout+yDelta))
        self.Bind(wx.EVT_BUTTON, self.OnResetClick, self.defaultButton)
        self.defaultButton.SetDefault()

        self.quickButton = wx.Button(panel, -1, "一键Monkey", pos=(xPos+120, yPoslayout+yDelta))
        self.Bind(wx.EVT_BUTTON, self.OnQuickStartClick, self.quickButton)
        self.quickButton.SetDefault()

        self.doButton = wx.Button(panel, -1, "开始Monkey", pos=(xPos+120*2, yPoslayout+yDelta))
        self.Bind(wx.EVT_BUTTON, self.OnStartClick, self.doButton)
        self.doButton.SetDefault()

        self.logButton = wx.Button(panel, -1, "生成Log", pos=(xPos, yPoslayout+2*yDelta))
        self.Bind(wx.EVT_BUTTON, self.OnBuildLog, self.logButton)
        self.logButton.SetDefault()

        self.stopButton = wx.Button(panel, -1, "停止Monkey", pos=(xPos+120, yPoslayout+2*yDelta))
        self.Bind(wx.EVT_BUTTON, self.OnStopMonkey, self.stopButton)
        self.stopButton.SetDefault()

        usr_home = os.path.expanduser('~')
        if (os.path.exists(usr_home+"/MonkeyLog")):
            print "MonkeyLog already exists"
        else:
            os.system("mkdir -p "+usr_home+"/MonkeyLog")

    def OnAction(self, event):
        value = self.seedCtrl.GetValue().strip()
        if all( x in '0123456789' for x in value):
            print value
            self.seedCtrl.SetValue(str(self.seedCtrl.GetValue()))

    def OnQuickStartClick(self, event):
        self.Reset()
        self.StartCmd()

    def OnSelectAllClick(self, event):
        listString = self.checkListBox
        count = listString.GetCount()
        array = []
        for i in range(0, count):
            array.append(i)
        listString.SetChecked(array)

    def OnUnselectClick(self, event):
        self.checkListBox.SetChecked([])

    def OnResetClick(self, event):
        self.Reset()

    def OnReadClick(self, event):
        self.checkListBox.Clear()
        os.system("adb shell ls data/data > ~/MonkeyLog/package.log")
        home = os.path.expanduser('~/MonkeyLog')
        f = open(home+"/package.log", 'r')
        line = f.readline()
        while line:
            line = f.readline()
            if (line != ""):
                print "===="+line
                self.checkListBox.Append(line)
        f.close()

    def OnStartClick(self, event):
        self.StartCmd()

    def Reset(self):
        self.ListFiles("/data/slog")
        self.seedCtrl.SetValue(self.seedDefault)
        self.excuteNumCtrl.SetValue(self.executionFrequencyDefault)
        self.delayNumCtrl.SetValue(self.delayDefault)
        self.excuteModeCtrl.SetSelection(5)
        self.logModeCtrl.SetSelection(2)

    def StartCmd(self):

        seed = self.seedCtrl.GetValue()
        excuteNum = self.excuteNumCtrl.GetValue()
        delayNum = self.delayNumCtrl.GetValue()
        excuteMode = self.excuteModeCtrl.GetValue()
        date = time.strftime('%Y%m%d%H%m%s',time.localtime(time.time()))
        listString = self.checkListBox

        package_section = ""
        package_list = listString.GetCheckedStrings()
        print "select package count:"+str(len(package_list))
        for i in range(0, len(package_list)):
            print package_list
            package = package_list[i]
            pack = package.strip('\r\n')
            package_section += (" -p "+pack)

        print package_section

        seed_section = " -s "+self.seedCtrl.GetValue()
        delay_section =" --throttle "+ delayNum
        log_section = ""
        mode_section = ""

        log_id = self.logModeCtrl.GetSelection()
        if (log_id == 0):
            log_section += " -v"
        elif (log_id == 1):
            log_section += " -v -v"
        elif (log_id == 2):
            log_section += " -v -v -v"


        mode_id = self.excuteModeCtrl.GetSelection()
        mode = [" --ignore-crashes ",
                " --ignore-timeouts ",
                " --ignore-security-exceptions ",
                " --ignore-native-crashes ",
                " --monitor-native-crashes "]
        if (mode_id == 0):
            mode_section = mode[0]
        elif (mode_id == 1):
            mode_section = mode[1]
        elif (mode_id == 2):
            mode_section = mode[2]
        elif (mode_id == 3):
            mode_section = mode[3]
        elif (mode_id == 4):
            mode_section = mode[4]
        else:
            mode_section = mode[0]+mode[1]+mode[2]+mode[3]+mode[4]

##############   create monkey log dir ###############

        usr_home = os.path.expanduser('~/MonkeyLog')
        os.chdir(usr_home)
        logDir = "monkeylog_"+date
        os.system("mkdir "+logDir)
        self.logDir = os.path.join(usr_home,logDir)
        print self.logDir
        os.chdir(logDir)

###############  record monkey trace ################

        monkeyCmd = "adb shell monkey "
        monkeyCmd = monkeyCmd+delay_section+seed_section+package_section+log_section+mode_section
        monkeyCmd = monkeyCmd + " "+excuteNum+" > trace.log"
        print monkeyCmd
        os.system(monkeyCmd)
        print '-----------monkey finish----------'

    def ListFiles(self,path):
        for root,dirs,files in os.walk(path):
            log_f = ""
            for f in files:
                if(f.find("main") == 0):
                    log_f = f.strip()
                    os.chdir(root)
                    if (log_f != ""):
                        grep_cmd = "grep -Eni -B20 -A20 'FATAL|error|exception|system.err|androidruntime' "+log_f+" > "+log_f+"_fatal.log"
                        os.system(grep_cmd)
        print "--------------finish build log-----------------"


    def BuildFatalLog(self,path):
        self.ListFiles(path)


    def OnBuildLog(self,event):
        log_home = os.path.expanduser('~/MonkeyLog')
        os.chdir(log_home)
        print self.logDir
        date = time.strftime('%Y-%m-%d-%H-%M',time.localtime(time.time()))
        dir_m = "monkey_log_"+date.replace("-","")
        dirl = "slog"

        if (os.path.exists(dir_m+"/"+dirl)):
            print "already exists"
        else:
            os.system("mkdir -p "+dir_m+"/"+dirl)

        os.chdir(dir_m)
        os.system("adb pull /data/slog "+dirl)
        self.BuildFatalLog(os.getcwd())

    def OnStopMonkey(self,event):
        os.system("adb shell ps | grep monkey | awk '{print$2}' | xargs adb shell kill")

if __name__ == '__main__':
    app = wx.PySimpleApp()
    frame = MyFrame()
    frame.Show(True)
    app.MainLoop()
