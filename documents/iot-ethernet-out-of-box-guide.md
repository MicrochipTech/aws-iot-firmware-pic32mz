### IoT Ethernet
#### Out of Box Guide

---
### Introduction
[AWS IoT](http://aws.amazon.com/iot/) is a service that will allow internet of things (IoT) devices to be easily and securely connected to Amazon Web Services (AWS).  The IoT Ethernet Kit has been design to work with this service and allow you to develop new IoT based designs.

![IoT Ethernet Overview](images/DM990004.png)

---
### Required Tools and Applications
#### Microchip Tools and Applications
You will need the following Microchip development tools to run out of box demo

- IoT Ethernet Kit (DM990004), available from [Microchip Direct](http://www.microchipdirect.com/productsearch.aspx?Keywords=DM990004)
- Download and install [Insight on Things](https://github.com/MicrochipTech/aws-iot-insight-on-things-desktop-app) desktop application; [Download latest version](https://github.com/MicrochipTech/aws-iot-insight-on-things-desktop-app/releases/latest)
- To learn more about building the project from source or just upgrading the firmware, read [IoT Ethernet Firmware Compiling and Programing](iot-ethernet-firmware-compile-and-program.md) in our documents folder

#### AWS Tools and Applications
- You will need an AWS account to use AWS IoT.
  - For more information how to setup an AWS Account please go to [AWS](http://aws.amazon.com) main page
  - Please check with your IT or IS department on your company's policies on using cloud computing, or to see if you have a corporate AWS account that should be used.

- To use the AWS IoT services you will need to make use of _AWS Command Line Interface_ (awscli) tool.  For more information on how to install, configure, and use the awscli tool please go to [AWS Command Line Interface User Guide](http://docs.aws.amazon.com/cli/latest/userguide/cli-chap-welcome.html).

> NOTE: If you are a **Microchip Employee** we have a corporate AWS account that must be used.  To setup your AWS IoT device please go to https://setup.iot.microchip.com for more information

---
### Setting Up the AWS IoT Service

To setup your AWS IoT service to work with the _IoT Ethernet Kit_ you will need to work with the awscli that you installed in the previous section.

1. Using the awscli you will need to find the endpoint that will be used for your AWS IoT account.  To do this run the following command in your terminal/command window.

  ```
  aws iot describe-endpoint
  ```
You will receive the following back that you will need to add to the firmware in the next section.  The output will look like: `<random-string>.iot.us-east-1.amazonaws.com`
- Create certificates that will be used in the application firmware by running the following command in your terminal/command prompt window:

  ```
  aws iot create-keys-and-certificate --set-as-active --output text
  ```
  > You will need to copy the output on the terminal/command window to text files for the certs and private key.  Please also note the _certificate arn_ value that will be used later in this section; it is the first long string of text that you will seen on the screen.
  >
  > NOTE: this is the only time that you will be able to see this information and make sure there is no text or spaces at the beginning of each of the files you create.  For example in the output below you will need to remove the `KEYPAIR` text and spaces so that the first line of the private key starts with the dashes:
  >
  >```
  >...
  >Gmh/2ib2NsA8Tf+8drTQXf3gQ4oemdOfnA4IlXiFGA9QAP9i/krzYvaVAlSogLWh
  >wRHaP1hzPuS9+0UjZkS1Uzyfqmd+vR4Om8S+t2VmEwYMJMG3mUaFXdhBhMDJ
  >-----END CERTIFICATE-----
  >
  >KEYPAIR  -----BEGIN RSA PRIVATE KEY-----
  >MIIEpAIBAAKCAQEAynvp1r7jl9HzbAVCgVWzYmKu4O6nNM4wBZQ1YjHpSOL1+joF
  >KKcXM8eS8jmUbHRJ84JeDrr9A50ok1cyUCDBPkf6c7VO4xWGzXruR3WoVjRYq3vc
  >...
  >```
- Create and attach the following policy that will allow your thing to publish and subscribe to the AWS IoT service
  - First create a file that will have the policy information in it and save it to you computer:
    > NOTE: Make sure you replace `<AWS-IoT Region>` and `<AWS Account Number>` with the AWS-IoT region and your AWS account number

    ```
    {
      "Version": "2012-10-17",
      "Statement": [{
        "Effect": "Allow",
        "Action": [
            "iot:Connect"
        ],
        "Resource": [
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:client/${iot:ClientId}"
        ]
      },
      {
        "Effect": "Allow",
        "Action": [
          "iot:Publish"
        ],
        "Resource": [
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/update",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/get"
        ]
      },
      {
        "Effect": "Allow",
        "Action": [
          "iot:Subscribe"
        ],
        "Resource": [
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topicfilter/$aws/things/${iot:ClientId}/shadow/update/delta",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topicfilter/$aws/things/${iot:ClientId}/shadow/update/accepted",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topicfilter/$aws/things/${iot:ClientId}/shadow/update/rejected",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topicfilter/$aws/things/${iot:ClientId}/shadow/get/accepted",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topicfilter/$aws/things/${iot:ClientId}/shadow/get/rejected"
        ]
      },
      {
        "Effect": "Allow",
        "Action": [
          "iot:Receive"
        ],
        "Resource": [
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/update/delta",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/update/accepted",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/update/rejected",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/get/accepted",
          "arn:aws:iot:<AWS-IoT Region>:<AWS Account Number>:topic/$aws/things/${iot:ClientId}/shadow/get/rejected"
        ]
      }]
    }
    ```
  - Load the policy to the AWS IoT services with the following command:

    ```
    aws iot create-policy --policy-name 'MQTTandShadowPolicy' --policy-document file://<path-to-your-policy-document>
    ```
  - You will need to attached this policy to a principal to allow the cert the rights to publish and subscribe to topics:

    ```
    aws iot attach-principal-policy --principal '<certificate-arn>' --policy-name 'MQTTandShadowPolicy'
    ```
- Create a thing shadow based on the MAC address of the board:

  ```
  aws iot create-thing --thing-name '<MAC Address of the board>'
  ```

---
### Running the Demo
There are two parts to running the demo.  First you have to commission the demo so it knows how to talk to your AWS Account that you setup in the [Setting Up the AWS IoT Service](Setting Up the AWS IoT Service) section of this out of box guide.  Second is sending and receiving data from AWS IoT.

#### Commissioning
To setup and run the demo follow these instructions:

1. Plug the IoT Ethernet Kit into a router or switch using an ethernet cable
- Power the IoT Ethernet Kit by connecting the supplied Micro-USB cable to a power source and setting the power switch on the board to USB
- Connect your computer to the same network that the IoT Ethernet Kit is connected
  - Your board will register a Multicast DNS (mDNS) name with the local network that will allow you to access a configuration webpage.
- Use a bonjour or multicast DNS service to find your board's IP address and enter the IP address into a web browser.  The kit will have a service name such as `xxxxxx_IoT-E` where `xxxxxx` is the last six characters of the kit's MAC address.
  - Alternatively you can try entering `xxxxxx_iot-e.local` into a web browser to navigate to the webpage where `xxxxxx` is the last six characters of the MAC address on the board
- Once the web browser finds and displays the IoT Ethernet Kit configuration webpage
  - Enter your AWS IoT Endpoint Address that you received from the previous AWS IoT Service Setup section.  The endpoint will have the form `<random-string>.iot.us-east-1.amazonaws.com`
- Copy and paste your AWS Certificate into the box labeled AWS Certificate.
  -  The cert should look like

    ```
    -----BEGIN CERTIFICATE-----
    <cert data>
    -----END CERTIFICATE-----
    ```
-  Copy and paste your AWS Certificate Private Key into the box labeled AWS Certificate Private Key.
  -  The key should look like

    ```
    -----BEGIN RSA PRIVATE KEY----
    <key data>
    -----END RSA PRIVATE KEY-----
    ```
- Click the __Join__ button and a _Connecting to server_ page will appear showing the endpoint that the IoT Ethernet board will connect to along with the AWS Thing Name (MAC Address).
  >  NOTE:  The information you enter above will be sent in the clear and is not secured as if you where using https; this is an option that you can add to your code later
- You will see a LEDs D1-D6 light up while the board gets initial data from the server.
  - If not, please see the [Status and Error Code Table](Status and Error Code Table) table below for more information
- The board is now connect and running; there are two blue LED status indicator on the board.
  - LED D6 will flash briefly when transmitting data
  - LED D5 will flash briefly when receiving data

#### Sending and Receiving Data with AWS IoT
This demo is controlled though the AWS IoT shadow registors by the [Insight on Things](https://github.com/MicrochipTech/aws-iot-insight-on-things-desktop-app) desktop application that we provide you.  [Click here download the latest version](https://github.com/MicrochipTech/aws-iot-insight-on-things-desktop-app/releases/latest).  You will need to follow the README.md guide on the github page to setup this application.

---

### Troubleshooting
If you are having trouble connecting with the starter kit, check to make sure that each of the issues bellow are resolved.

#### Connection Issue
- Check that you have a valid internet connection on your network.
- Ensure that port 8883 is open to the internet.
- Ensure that your AWS IoT service is setup property.

#### Change the configuration
- Changing the configuration that you entered, or if you entered it incorrectly, simply power off the starter kit, press and hold S2 and S3 while turning on the power to the starter kit.
- This will erase the current configuration and you can begin this process again and re-enter the configuration information.


#### Association to AP Failed
- Check that your access point is not blocking MAC addresses.

#### DNS Resolved Failed
- Check that the AWS IoT Endpoint Address you configured your starter kit with is correct.
- Check that you have a valid internet connection on your network

#### Status and Error Code Table
 D7  | D6  | D5  | D4  | D3  | D2  | D1  | Diagnostic Report
:---:|:---:|:---:|:---:|:---:|:---:|:---:|-----
 -   | 0   | F   | 0   | 0   | 0   | 0   | Configuration Mode: Need to configure Connected to network
 -   | 1   | 1   | 1   | 1   | 1   | 1   | Connected to network and waiting on data
 -   | 1   | 0   | C   | C   | C   | C   | Associating to network
 -   | 1   | 1   | 0   | 0   | 0   | 1   | Connection Issue, no internet found
 -   | 1   | 1   | 0   | 0   | 1   | 0   | Association to network failed
 -   | 1   | 1   | 0   | 1   | 0   | 1   | DNS Unresolved: Bad endpoint or no connection
 -   | F   | 0   | -   | -   | -   | -   | Normal Operation: D6 flashes each message transmission
 -   | 0   | F   | -   | -   | -   | -   | Normal Operation: D5 flashes each subscription receive
 1   | -   | -   | -   | -   | -   | -   | Battery Voltage Low
F = Flashing, C = Rotating in a counter clockwise pattern, - = Don’t care, 0 = Off, and 1 = On
