response_times = textread( 'message_response_times.txt', '%u', 'delimiter', '\n' );
arrival_times = textread( 'message_arrival_times.txt', '%u', 'delimiter', '\n' );
dispatch_times = textread( 'message_dispatch_times.txt', '%u', 'delimiter', '\n' );

n_samples = 50;

data = zeros( n_samples, 2 );

for i=1:n_samples
    data( i, 1 ) = arrival_times( i ) - dispatch_times( i );
    %data( i, 2 ) = response_times( i ) - arrival_times( i );
    data( i, 2 ) = response_times( i ) - dispatch_times( i );
    %data( i, 2 ) = data( i, 2 ) - data( i, 1 );
end

b = bar( data, 2.0 );
%b = bar( data, 'stacked' );
set( b(2), 'FaceColor', 'yellow' );

axis( [ 0 n_samples+1 0 400 ] );
xlabel( 'Ordem de envios de mensagem inicial' );
ylabel( 'Tempo de resposta (ms)' );