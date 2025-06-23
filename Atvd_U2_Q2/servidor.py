import socket

# Configure com o mesmo IP e Porta do código da Pico
# Use '0.0.0.0' para escutar em todas as interfaces de rede do PC
HOST = '192.168.1.106'
PORT = 5555

# Cria um socket UDP
# AF_INET para IPv4, SOCK_DGRAM para UDP
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    # Associa o socket ao endereço e porta
    s.bind((HOST, PORT))
    print(f"Servidor UDP escutando em {HOST}:{PORT}")

    # Loop infinito para receber dados
    while True:
        # Espera por dados, o buffer de 1024 bytes é suficiente
        data, addr = s.recvfrom(1024)
        print(f"Recebido de {addr}: {data.decode('utf-8')}")
