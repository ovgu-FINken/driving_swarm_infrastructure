import subprocess
import re

def __get_ips():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    return [ip[5:] for ip in ips]


def get_ip():
    ips = __get_ips()
    return ", ".join(ips)


def get_robot_name(prefix):
    ips = __get_ips()
    x = [ip.split('.') for ip in ips]
    name = prefix
    for ip in x:
        if ip[0] != '127':
            name += ip[-1]
    return name

    """
    def get_ip_name():
    ipa = subprocess.check_output(['ip', 'a']).decode('ascii')
    ips = re.findall(r'inet\s[0-9.]+', ipa)
    ips = [ip[5:] for ip in ips]
    x = [ip.split('.') for ip in ips]
    name = 'T'
    for ip in x:
        if ip[0] != '127':
            name += ip[-1]
    return name
    """

def get_robot_name_from_msg(ipFromMsg):
    x = [ipFromMsg.split('.')]
    local_ip = ''
    for ip in x:
        # print(ip) # e.g. ['127', '0', '0', '1, 10', '61', '10', '245']
        if ip[0] == '127':
            local_ip += ip[-1]
    return local_ip
