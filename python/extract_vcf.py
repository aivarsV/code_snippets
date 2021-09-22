#!/usr/bin/env python3
# Script extracts vcf contacts file to simple text file

import sys
import re
import os

def main(args):
    inFilePath = args[0]
    countryCode = "" if len(args) < 2 else args[1]

    # Storing in dict will filter out duplicate contacts
    allContacts = {}

    with open(inFilePath, "r") as inFile:
        l = inFile.readline()
        while l != "":
            l = l.strip()
            if l == "BEGIN:VCARD":
                contactName = None
                numbers = []
            if l.startswith("FN:") or l.startswith("FN;"):
                contactName = l.split(":")[-1]
                if "ENCODING=QUOTED-PRINTABLE:" in l:
                    encoding = re.search(r'CHARSET=([^;:]*)', l)
                    if encoding:
                        contactName = bytes.fromhex(contactName.replace("=", "")).decode(encoding.group(1))

            if l.startswith("TEL"):
                number = l.split(":")[-1]
                numbers.append(number if number.startswith("+") else countryCode + number)

            if l == "END:VCARD":
                for n in numbers:
                    allContacts[n] = contactName

            l = inFile.readline()

    outFilePath = ".".join(inFilePath.split(".")[:-1]) + ".txt"

    if os.path.isfile(outFilePath):
        os.remove(outFilePath)

    with open(outFilePath, "w") as outFile:
        for number, name in allContacts.items():
            outFile.write(number + ">" + name + "\n")


if __name__ == "__main__":
    main(sys.argv[1:])
