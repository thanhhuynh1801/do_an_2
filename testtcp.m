% Địa chỉ IP và cổng của máy chủ
serverIP = '127.0.0.1';  % Địa chỉ IP của máy chủ
serverPort = 5000;       % Cổng của máy chủ

% Tạo kết nối TCP/IP với máy chủ
tcpClient = tcpip(serverIP, serverPort);
set(tcpClient, 'Timeout', 10);  % Đặt thời gian chờ tối đa cho kết nối

% Kết nối tới máy chủ
disp('Đang kết nối tới máy chủ...');
fopen(tcpClient);
disp('Đã kết nối thành công tới máy chủ!');

% Gửi dữ liệu tới máy chủ
dataToSend = 'Hello, server!';  % Dữ liệu để gửi
fwrite(tcpClient, dataToSend);

% Đọc dữ liệu từ máy chủ
dataReceived = fread(tcpClient, tcpClient.BytesAvailable);
disp('Dữ liệu từ máy chủ:');
disp(char(dataReceived'));

% Đóng kết nối
fclose(tcpClient);