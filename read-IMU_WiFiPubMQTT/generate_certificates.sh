#!/bin/bash

# Generate self-signed certificates for MQTT SSL testing
# This script creates the necessary certificates for testing the SSL connection

echo "Generating self-signed certificates for MQTT SSL testing..."
echo "================================================="

# Create certificates directory
mkdir -p certificates
cd certificates

# Generate CA private key
echo "1. Generating CA private key..."
openssl genrsa -out ca.key 2048

# Generate CA certificate
echo "2. Generating CA certificate..."
openssl req -new -x509 -days 365 -key ca.key -out ca.crt -subj "/C=IT/ST=Friuli/L=Udine/O=UNIUD/OU=Tirocinio/CN=mqtt-ca"

# Generate server private key
echo "3. Generating server private key..."
openssl genrsa -out server.key 2048

# Generate server certificate signing request
echo "4. Generating server certificate signing request..."
openssl req -new -key server.key -out server.csr -subj "/C=IT/ST=Friuli/L=Udine/O=UNIUD/OU=Tirocinio/CN=192.168.137.212"

# Generate server certificate
echo "5. Generating server certificate..."
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 365

# Generate client private key (optional)
echo "6. Generating client private key..."
openssl genrsa -out client.key 2048

# Generate client certificate signing request
echo "7. Generating client certificate signing request..."
openssl req -new -key client.key -out client.csr -subj "/C=IT/ST=Friuli/L=Udine/O=UNIUD/OU=Tirocinio/CN=mqtt-client"

# Generate client certificate
echo "8. Generating client certificate..."
openssl x509 -req -in client.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out client.crt -days 365

# Clean up CSR files
rm server.csr client.csr

echo ""
echo "Certificate generation complete!"
echo "Files created in certificates/ directory:"
echo "- ca.crt (Certificate Authority)"
echo "- ca.key (CA private key)"
echo "- server.crt (Server certificate)"
echo "- server.key (Server private key)"
echo "- client.crt (Client certificate)"
echo "- client.key (Client private key)"
echo ""
echo "To use these certificates:"
echo "1. Copy server.crt and server.key to your Mosquitto broker"
echo "2. Update mosquitto_ssl.conf with the correct paths"
echo "3. Restart Mosquitto broker"
echo "4. Update the Python script if using client certificates"
echo ""
echo "Note: These are self-signed certificates for testing only!"
echo "For production use, obtain certificates from a trusted CA."
