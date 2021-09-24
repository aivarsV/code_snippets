#!/usr/bin/env python3
# Script Allows to send sms without using that annoying HiLink web interface

import requests
import sys
import re
from socket import gethostbyname_ex

payload_formatter = """
<?xml version="1.0" encoding="UTF-8"?>
<request>
  <Index>-1</Index>
  <Phones>
    <Phone>{recipient}</Phone>
  </Phones>
  <Sca></Sca>
  <Content>{content}</Content>
  <Length>{len}</Length>
  <Reserved>1</Reserved>
  <Date>2021-09-23 20:14:10</Date>
</request>
"""

def main(address, recipient, contents):
    server_address = gethostbyname_ex(address)[-1][0]
    session = requests.Session()
    inbox_link = f'http://{server_address}/html/smsinbox.html'
    html_text = session.get(inbox_link).text
    tokens = re.findall(r'(?:<meta name="csrf_token" content="([-A-Za-z0-9+/=]*)"\/>)', html_text)

    payload = payload_formatter.format(recipient=recipient, content=contents, len=len(contents))
    resp = session.post(f'http://{server_address}/api/sms/send-sms',
                            data=payload.encode("utf-8"),
                            headers={'__RequestVerificationToken':tokens[0]})

    resp.encoding = 'utf-8'

    print("({0})<='{1}'".format(recipient, contents))
    print(resp.text)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], " ".join(sys.argv[3:]))
