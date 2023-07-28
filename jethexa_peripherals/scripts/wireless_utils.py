import subprocess


def split_to_dict(info):
    info_dict = {}
    for i in info:
        if ',' in i:
            j = i.split(',')
            j = split_to_dict(j)
            info_dict.update(j)
        else:
            j = i.split(' ')
            if len(j)>= 2:
                info_dict[j[0]] = ''.join(j[1:])
    return info_dict

def dev_info(ifname):
    cmd = "iw dev {} info".format(ifname)
    info = subprocess.check_output(cmd, shell=True)
    info = str(info, encoding='utf8').replace('\t', '').replace(':', '').replace(', ', ',').split('\n')
    info = split_to_dict(info)
    return info

def dev_link(ifname):
    cmd = "iw dev {} link".format(ifname)
    link = subprocess.check_output(cmd, shell=True)
    link = str(link, encoding='utf8').replace('\t', '').replace(':', '').replace(', ', ',').split('\n')
    link = split_to_dict(link)
    return link

def dev_state(ifname):
    state = {'mode':'None', 'ssid':'None'}
    info = dev_info(ifname)
    mode = 'STA' if info['type'] != 'AP' else 'AP'
    if mode == 'AP':
        if 'ssid' in info:
            state['ssid'] = info['ssid']
            state['mode'] = 'AP'
    else:
        link = dev_link(ifname)
        if 'SSID' in link:
            state['ssid'] = dev_link(ifname)['SSID']
            state['mode'] = 'STA'
    return state

if __name__ == '__main__':
    a = dev_state('wlan0')
    print(a)