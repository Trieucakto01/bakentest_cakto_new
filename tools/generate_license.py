#!/usr/bin/env python3
import argparse

SECRET = bytes.fromhex("a142aeaa6d61f676b684daba8f437280")
MASK = (1 << 64) - 1

def rotl(v, n): return ((v << n) | (v >> (64 - n))) & MASK
def rnd(v0, v1, v2, v3):
    v0 = (v0 + v1) & MASK; v1 = rotl(v1, 13) ^ v0; v0 = rotl(v0, 32)
    v2 = (v2 + v3) & MASK; v3 = rotl(v3, 16) ^ v2
    v0 = (v0 + v3) & MASK; v3 = rotl(v3, 21) ^ v0
    v2 = (v2 + v1) & MASK; v1 = rotl(v1, 17) ^ v2; v2 = rotl(v2, 32)
    return v0, v1, v2, v3
def siphash(data):
    k0, k1 = int.from_bytes(SECRET[:8], "little"), int.from_bytes(SECRET[8:], "little")
    v0, v1 = 0x736F6D6570736575 ^ k0, 0x646F72616E646F6D ^ k1
    v2, v3 = 0x6C7967656E657261 ^ k0, 0x7465646279746573 ^ k1
    end = len(data) - len(data) % 8
    for off in range(0, end, 8):
        m = int.from_bytes(data[off:off + 8], "little")
        v3 ^= m; v0, v1, v2, v3 = rnd(v0, v1, v2, v3); v0, v1, v2, v3 = rnd(v0, v1, v2, v3); v0 ^= m
    tail = len(data) << 56
    for i, b in enumerate(data[end:]): tail |= b << (8 * i)
    v3 ^= tail; v0, v1, v2, v3 = rnd(v0, v1, v2, v3); v0, v1, v2, v3 = rnd(v0, v1, v2, v3); v0 ^= tail
    v2 ^= 0xFF
    for _ in range(4): v0, v1, v2, v3 = rnd(v0, v1, v2, v3)
    return v0 ^ v1 ^ v2 ^ v3
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("device_id")
    device = "".join(c for c in parser.parse_args().device_id.upper() if c in "0123456789ABCDEF")
    if len(device) != 12: raise SystemExit("Device ID must contain 12 hex characters")
    print(f"{siphash(device.encode()):016X}")
if __name__ == "__main__": main()
