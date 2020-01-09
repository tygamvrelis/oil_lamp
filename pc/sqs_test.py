# https://boto3.amazonaws.com/v1/documentation/api/latest/guide/sqs-example-sending-receiving-msgs.html
# https://boto3.amazonaws.com/v1/documentation/api/latest/guide/sqs-example-using-queues.html
import boto3
from decimal import Decimal
import time
from random import random

# Create SQS client
sqs = boto3.client('sqs')

# Get URL for SQS queue
queue_url = sqs.get_queue_url(QueueName='LampQ.fifo')['QueueUrl']

# Send message to SQS queue
response = sqs.send_message(
    QueueUrl=queue_url,
    DelaySeconds=0,
    MessageAttributes={
        'Outer': {
            'DataType': 'Binary',
            'BinaryValue': str(12.5).encode()
        },
        'Inner': {
            'DataType': 'Binary',
            'BinaryValue': str(-31.23902392948).encode()
        }
    },
    MessageBody=(' '),
    MessageGroupId='0',
    MessageDeduplicationId='0'
)

print(response['MessageId'])