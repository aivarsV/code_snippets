#!/usr/bin/env python3
# Script fetches received sms messages from Huawei HiLink web interface
#   and stores them in atom feed

import requests
import re
import xml.etree.ElementTree as ET
import sys
from socket import gethostbyname_ex
from datetime import datetime, timezone
from uuid import uuid4

post_payload = b"""
<?xml version="1.0" encoding="UTF-8"?>
<request>
    <PageIndex>1</PageIndex>
    <ReadCount>50</ReadCount>
    <BoxType>1</BoxType>
    <SortType>0</SortType>
    <Ascending>0</Ascending>
    <UnreadPreferred>0</UnreadPreferred>
</request>
"""


sample_feed = """<?xml version="1.0" encoding="UTF-8" ?>
<feed xmlns="http://www.w3.org/2005/Atom">
    <title>Messages from Huawei modem</title>
    <id>urn:uuid:60a76c80-d399-11d9-b91C-0003939e0af6</id>
    <updated>2002-10-02T10:00:00-05:00</updated>
</feed>
"""


def main(address, atom_file, contactsFile=None):
    contacts = None
    server_address = gethostbyname_ex(address)[-1][0]
    session = requests.Session()
    inbox_link = f'http://{server_address}/html/smsinbox.html'
    html_text = session.get(inbox_link).text
    tokens = re.findall(r'(?:<meta name="csrf_token" content="([-A-Za-z0-9+/=]*)"\/>)', html_text)

    resp = session.post(f'http://{server_address}/api/sms/sms-list',
                            data=post_payload,
                            headers={'__RequestVerificationToken':tokens[0]})

    resp.encoding = 'utf-8'

    sms_parsed_list = ET.fromstring(resp.text)

    ns = '{http://www.w3.org/2005/Atom}'
    ET.register_namespace('', ns[1:-1])

    try:
        atom_feed = ET.parse(atom_file).getroot()
    except Exception:
        atom_feed = ET.fromstring(sample_feed)
        atom_feed.find(ns + 'id').text = 'urn:uuid:{}'.format(uuid4())

    latest_date = datetime.fromisoformat(atom_feed.findtext(ns + 'updated'))

    tzone = timezone(datetime.now().replace(second=0, microsecond=0) -
                    datetime.utcnow().replace(second=0, microsecond=0))

    feed_entries = atom_feed.findall(ns + 'entry')
    for entry in feed_entries:
        atom_feed.remove(entry)

    feed_updated = False

    for message in sms_parsed_list.find('Messages').findall('Message'):
        if contacts is None:
            if contactsFile is not None:
                with open(contactsFile, 'r') as f:
                    contacts = f.readlines()

            else:
                contacts = []

        content = message.findtext('Content')
        sender = message.findtext('Phone')
        if sender.startswith('+'):
            for c in contacts:
                if c.startswith(sender):
                    sender = c[len(sender) + 1:].strip()

        title = '({0})=>{1}'.format(sender, content[0:min(len(content), 20)])
        received = datetime.fromisoformat(message.findtext('Date')).replace(tzinfo=tzone)

        latest_date = max(latest_date, received)
        existing_entry = None
        for entry in feed_entries:
            if entry.findtext(ns + 'content') != content:
                continue
            if entry.findtext(ns + 'title') != title:
                continue
            if entry.findtext(ns + 'updated') != received.isoformat():
                continue

            existing_entry = entry
            feed_entries.remove(entry)
            break

        if existing_entry is None:

            entry = ET.Element('entry')
            e = ET.Element('title')
            e.text = title
            entry.append(e)

            e = ET.Element('id')
            e.text = 'urn:uuid:{}'.format(uuid4())
            entry.append(e)

            e = ET.Element('updated')
            e.text = received.isoformat()
            entry.append(e)

            author = ET.Element('author')
            e = ET.Element('name')
            e.text = sender
            author.append(e)
            entry.append(author)

            e = ET.Element('updated')
            e.text = received.isoformat()
            entry.append(e)

            e = ET.Element('content')
            e.text = content
            entry.append(e)

            atom_feed.append(entry)
            feed_updated = True
        else:
            atom_feed.append(existing_entry)

    if feed_updated:
        ET.ElementTree(atom_feed).write(atom_file)



if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
