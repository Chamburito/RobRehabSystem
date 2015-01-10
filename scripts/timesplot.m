response_times = textread( '../data/message_response_times.txt', '%u', 'delimiter', '\n' );
arrival_times = textread( '../data/message_arrival_times.txt', '%u', 'delimiter', '\n' );
dispatch_times = textread( '../data/message_dispatch_times.txt', '%u', 'delimiter', '\n' );

send_calls_server = textread( '../data/send_calls_server.txt', '%u', 'delimiter', '\n' );
send_calls_client = textread( '../data/send_calls_client.txt', '%u', 'delimiter', '\n' );
receive_calls_server = textread( '../data/receive_calls_server.txt', '%u', 'delimiter', '\n' );
receive_calls_client = textread( '../data/receive_calls_client.txt', '%u', 'delimiter', '\n' );

n_samples = 80;

time_data = zeros( n_samples, 2 );
for i=1:n_samples
    time_data( i, 1 ) = arrival_times( i ) - dispatch_times( i );
    time_data( i, 2 ) = response_times( i ) - arrival_times( i );
end

calls_data = zeros( n_samples, 4 );
for i=1:n_samples
    calls_data( i, 1 ) = send_calls_client( i );
    calls_data( i, 2 ) = receive_calls_server( i );
    calls_data( i, 3 ) = send_calls_server( i );
    calls_data( i, 4 ) = receive_calls_client( i );
end

figure(1), b = bar( time_data, 'stacked' );
set( b(1), 'FaceColor', 'blue', 'EdgeColor', 'black' );
set( b(2), 'FaceColor', 'yellow', 'EdgeColor', 'black' );
axis( [ 0 n_samples+1 0 400 ] );
xlabel( 'Ordem de envios de mensagem inicial' );
ylabel( 'Tempo de resposta (ms)' );

figure(2), b = bar( calls_data, 'stacked' );
set( b(1), 'FaceColor', 'blue', 'EdgeColor', 'black' );
set( b(2), 'FaceColor', 'green', 'EdgeColor', 'black' );
set( b(3), 'FaceColor', 'red', 'EdgeColor', 'black' );
set( b(4), 'FaceColor', 'yellow', 'EdgeColor', 'black' );
axis( [ 0 n_samples+1 0 400 ] );
xlabel( 'Ordem de envios de mensagem inicial' );
ylabel( 'Número de chamadas' );