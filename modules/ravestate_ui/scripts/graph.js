    var socket = io.connect('http://' + document.domain + ':' + location.port);
    var data;
    socket.on('newdata', function(newdata) {
        data = newdata
        console.log("Hello");
    });

    var nodes;
    var activeNode;

    $(function () {
        $.getJSON("/data", function (data) {

            // Expected input data format:
            //     [{source: "property_name", target: "signal_name", type: "sets"},]
            // Types of links:
            //     - changes: state -> property
            //     - triggers: signal -> state
            //     - emits: state -> signal
            //     - sets: property -> signal

            var links = data;
            var signals = {};
            var states = {};
            var properties = {};

            // Compute the distinct nodes from the links.
            var i = 0;
            var j = 0;
            links.forEach(function (link) {
                if (link.type === 'changes') { // state -> prop
                    link.source = states[link.source] || (states[link.source] = {name: link.source, index: i++, weight: j++});
                    link.target = properties[link.target] || (properties[link.target] = {name: link.target, index: i++, weight: j++});
                } else if (link.type === 'triggers') { // signal -> state
                    link.source = signals[link.source] || (signals[link.source] = {name: link.source, index: i++, weight: j++});
                    link.target = states[link.target] || (states[link.target] = {name: link.target, index: i++, weight: j++});
                } else if (link.type === 'emits') { // state -> signal
                    link.source = states[link.source] || (states[link.source] = {name: link.source, index: i++, weight: j++});
                    link.target = signals[link.target] || (signals[link.target] = {name: link.target, index: i++, weight: j++});
                } else if (link.type === 'sets') { // prop -> signal
                    link.source = properties[link.source] || (properties[link.source] = {name: link.source, index: i++, weight: j++});
                    link.target = signals[link.target] || (signals[link.target] = {name: link.target, index: i++, weight: j++});
                } else {
                    console.log(link);
                }
            });

            nodes = Object.assign({}, signals, states, properties);
            var width = window.innerWidth,
                height = window.innerHeight;
            var force = d3.layout.force()
                .nodes(d3.values(nodes))
                .links(links)
                .size([width, height])
                .linkDistance(height/6)
                .charge(-250)
                .gravity(0.042)
                .on("tick", tick)
                .start();


            // ---------- Setup graph wrapper ------------
            var svg = d3.select("#chart")
                .classed("svg-container", true)
                .append("svg")
                .attr("preserveAspectRatio", "xMinYMin meet")
                .attr("viewBox", "0 0 " + width + " " + height)
                .classed("svg-content-responsive", true);
                // add to css:
                // overflow-y: scroll;
            // overhiflow-x: scroll;

// Remove resizing for scrollbars
//            const resizeHandler = () => {
//                const wrapper = document.querySelector('.svg-container');
//                document.querySelector('.svg-container > svg')
//                    .setAttribute('viewBox', "0 0 " +
//                        wrapper.getBoundingClientRect().width + " " +
//                        wrapper.getBoundingClientRect().height)
//            };
//            resizeHandler();
//            window.addEventListener('resize', resizeHandler)


            // Per-type markers, as they don't inherit styles.
            svg.append("defs").selectAll("marker")
                .data(["triggers"])
                .enter().append("marker")
                .attr("id", function (d) {
                    return d;
                })
                .attr("viewBox", "0 -5 10 10")
                .attr("refX", 42) // Increase with larger circle radius
                .attr("refY", -4)  // Decrease with increasing radius to counter arc
                .attr("markerWidth", 10)
                .attr("markerHeight", 10)
                .attr("orient", "auto")
                .attr("fill", "#ffffff")
                .append("path")
                .attr("d", "M0,-5L10,0L0,5");

            var path = svg.append("g").selectAll("path")
                .data(force.links())
                .enter().append("path")
                .attr("class", function (d) {
                    return "link " + d.type;
                })
                .attr("marker-end", function (d) {
                    return "url(#" + d.type + ")";
                });

            var circle = svg.append("g").selectAll("circle")
                .data(force.nodes())
                .enter().append("circle")
                .attr("r", 50)
                .attr("r", function (d) {
                    return (d.name in states) ? 50 : 30})
                .attr("class", function (d) {
                    return (d.name in signals) ? "signal" :
                        ((d.name in states) ? "state" : "prop")})
                .attr("nodeName", function (d) {
                    return d.name
                })
                .call(force.drag);

            var text = svg.append("g").selectAll("text")
                .data(force.nodes())
                .enter().append("text")
                .attr("x", -43)
                .attr("y", 0)
                .text(function (d) {
                    return d.name;
                });

            // Use elliptical arc path segments to doubly-encode directionality.
            function tick() {
                path.attr("d", linkArc);
                circle.attr("transform", transform);
                text.attr("transform", transform);
            }

            function linkArc(d) {
                var dx = d.target.x - d.source.x,
                    dy = d.target.y - d.source.y,
                    dr = Math.sqrt(dx * dx + dy * dy);
                return "M" + d.source.x + "," + d.source.y + "A" + dr + "," + dr + " 0 0,1 " + d.target.x + "," + d.target.y;
            }

            function transform(d) {
                return "translate(" + d.x + "," + d.y + ")";
            }

            socket.on('activate', function(stateName){
                if(activeNode) {
                    activeNode.attr("democolor", "off");
                }
                var activaaate =d3.select("[nodeName=\"" + stateName + "\"]");
                activaaate.attr("democolor", "on");
                activeNode = activaaate;
            });


            socket.on('spike', function(stateName){
                $("circle").one('animationiteration webkitAnimationIteration', function() {
                     $(this).removeClass("spiking");
                });
                // d3.selectAll("circle").attr("spiking", "off");
                d3.select("[nodeName=\"" + stateName + "\"]").attr("spiking", "on");
            });

        });

    })
