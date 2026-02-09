Secure File Sharing & Network Architecture Design

This project presents a full end-to-end network architecture designed for a growing boutique fitness studio, with an emphasis on security, scalability, and operational reliability. The system models a real-world small enterprise environment that integrates staff operations, point-of-sale systems, IoT-enabled equipment, guest access, surveillance infrastructure, and cloud-based services into a segmented and secure network design.

Problem Context
The business required:
- Secure handling of sensitive client and payment data
- High availability for booking and operational systems
- Clear separation between operational, guest, and IoT traffic
- A network design that could scale across multiple physical locations

System Design Overview
The architecture uses a layered enterprise-style design, including:
- VLAN-based network segmentation (Staff, POS, IoT, Guest, CCTV, Management)
- Router-on-a-stick inter-VLAN routing
- Dual-WAN redundancy for high uptime
- Secure Wi-Fi separation for staff and guests
- Hybrid cloud architecture for booking, storage, and backups
Cloud infrastructure was designed using AWS (Lightsail, S3, Glacier, Route 53), with integrated security controls including encryption, IAM policies, and DDoS protection.

Security & Reliability
Security was treated as a core system constraint rather than an afterthought:
- PCI-aware POS isolation
- Strict inter-VLAN access controls
- Encrypted VPN access for remote administration
- Encrypted cloud storage and automated backups
- Physical security integration via CCTV and local NVR storage

Validation
The design was implemented and validated using Cisco Packet Tracer, including:
- VLAN configuration and trunking
- Inter-VLAN routing verification
- Connectivity and isolation testing across network segments

Key Takeaway
This project emphasizes systems-level thinking: translating business requirements into a secure, scalable technical architecture while balancing cost, performance, and operational risk.
