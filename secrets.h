// secrets.h
#ifndef SECRETS_H
#define SECRETS_H

#define WIFI_SSID "ENTER SSID"
#define WIFI_PASSWORD "ENTER PASSWORD"

// AWS IoT details
#define AWS_IOT_ENDPOINT "your-aws-endpoint.amazonaws.com"
#define AWS_IOT_CLIENT_ID "your-client-id"
#define AWS_IOT_TOPIC "your/topic"

// Certificate and private key for AWS IoT (these should be securely stored)
const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
Your-CA-certificate-content
-----END CERTIFICATE-----
)EOF";

const char AWS_CERT_CRT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
Your-certificate-content
-----END CERTIFICATE-----
)EOF";

const char AWS_CERT_PRIVATE[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
Your-private-key-content
-----END RSA PRIVATE KEY-----
)EOF";

#endif // SECRETS_H
