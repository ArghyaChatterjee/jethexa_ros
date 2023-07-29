#!/usr/bin/env python3
# coding=utf8
import time
import smbus  #调用树莓派IIC库
#幻尔科技语音合成模块使用例程#

class TTS:
      
    address = 0x40
    bus = None

    def __init__(self, bus=1):
        self.bus = smbus.SMBus(bus)
    
    def WireReadTTSDataByte(self):
        try:
            val = self.bus.read_byte(self.address)
        except:
            return False
        return True
    
    def TTSModuleSpeak(self, sign, words):
        head = [0xFD,0x00,0x00,0x01,0x00]             #文本播放命令
        wordslist = words.encode("gb2312")            #将文本编码格式设为GB2312
        signdata = sign.encode("gb2312")    
        length = len(signdata) + len(wordslist) + 2
        head[1] = length >> 8
        head[2] = length
        head.extend(list(signdata))
        head.extend(list(wordslist))       
        try:
            self.bus.write_i2c_block_data(self.address, 0, head) #向从机发送数据
        except:
            print('Sensor not connected!')
        time.sleep(0.05)
        
if __name__ == '__main__':
    v = TTS()
    #[h0]设置单词发音方式，0为自动判断单词发音方式，1为字母发音方式，2为单词发音方式
    #[v10]设置音量，音量范围为0-10,10为最大音量。
    #[m53]选择发音人，3为小燕（女），51为许久（男）， 52为许多（男）， 53为小萍（女）
    #更多方法请参考手册
    v.TTSModuleSpeak("[h0][v10][m53]","你好")   
    #注意括号里的字符长度不能超过32,如果超过了请分多次来说
    time.sleep(1) #要适当延时等说话完成
