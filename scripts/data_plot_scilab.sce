arrival_times = fscanfMat( '../data/message_arrival_times.txt' );
response_times = fscanfMat( '../data/message_response_times.txt' );
dispatch_times = fscanfMat( '../data/message_dispatch_times.txt' );

receive_calls_server = fscanfMat( '../data/receive_calls_server.txt' );
receive_calls_client = fscanfMat( '../data/receive_calls_client.txt' );
send_calls_server = fscanfMat( '../data/send_calls_server.txt' );
send_calls_client = fscanfMat( '../data/send_calls_client.txt' );

n_samples = 50;

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

figure( 0 );
bar( time_data, 'stacked' );

figure( 1 );
bar( calls_data, 'stacked' );
