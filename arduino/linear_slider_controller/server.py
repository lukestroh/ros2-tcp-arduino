import asyncio
import sys

# Change this for host computer
HOST = '169.254.93.234'
ARDUINO_PORT = 11412


async def handle_echo(reader, writer):
    # Get "<Arduino is ready> message"
    addr = writer.get_extra_info('peername')
    print(f'Got connection {addr}'.replace('\r','').replace('\n',''))

    # Input a command, get a response
    try:
        while True:
            # send_data = input("Stepper position: ")
            # send_data = '<'+str(send_data)+'>'
            # writer.write(send_data.encode('utf-8'))
            # await writer.drain()

            # await asyncio.sleep(0.01)
            read_data = (await reader.read(1024)).decode('utf-8')
            addr = writer.get_extra_info('peername')
            if read_data:
                print(f"{read_data} ----------- from {addr}".replace('\r','').replace('\n',''))

    except (KeyboardInterrupt, ConnectionResetError) as e:
        print("exit 1")
        print(e)
    finally:
        # Close the connection
        writer.close()
        await writer.wait_closed()
        sys.exit()

async def ask_exit():
    for task in asyncio.all_tasks():
        task.cancel()
    return

async def main():
    try:
        server = await asyncio.start_server(
            client_connected_cb=handle_echo, 
            host=HOST, 
            port=ARDUINO_PORT
        )

        # Get address from client
        addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
        print(f'Serving on {addrs}')

        async with server:
            await server.serve_forever()
    except (Exception) as e:
        print(e)
    finally:
        server.close()
        await server.wait_closed()
        print("exit 2")
        sys.exit()

if __name__ == '__main__':
    asyncio.run(main())