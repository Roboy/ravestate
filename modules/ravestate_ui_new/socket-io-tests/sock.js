const io = require('socket.io')(4242);
io.origins('*:*');

let counter = 0;

io.on('connection', function (socket) {
    counter++;
    console.log('client has connected', counter);
    io.emit('hello', { clientID: counter});

    socket.on('a', msg => {
        console.log('received msg A', msg);
        io.emit('reply', 'aa');
    });

    socket.on('b', msg => {
        console.log('received msg B', msg);
        io.emit('reply', 'bb');
    });
});