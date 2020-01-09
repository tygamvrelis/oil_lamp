import boto3
from decimal import Decimal
import time
from random import random

# Get the service resource.
dynamodb = boto3.resource('dynamodb')

table = dynamodb.Table('Lamp')


# for i in range(200):
#     table.put_item(
#     Item={
#             'SeqNumber': str(i),
#             'outer': Decimal(str(2*(random() - 0.5) * 40)),
#             'inner': Decimal(str(2*(random() - 0.5) * 40)),
#         }
#     )

# Batch writer has less latency...
# So we should be buffering our data before writing it. Maybe when the playback
# application starts up, it can buffer 2-3 seconds worth of data for playback,
# and the sensor application can put data in the DB every second or so
with table.batch_writer() as batch:
    for i in range(200):
        batch.put_item(
        Item={
                'SeqNumber': str(i),
                'outer': Decimal(str(2*(random() - 0.5) * 40)),
                'inner': Decimal(str(2*(random() - 0.5) * 40)),
            }
        )

print("Done Send")

response = table.get_item(
    Key={
        'SeqNumber': '4'
    }
)
item = response['Item']
print(item)
