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
                .linkDistance(height/5)
                .charge(-200)
                .gravity(0.02)
                .on("tick", tick)
                .start();


            // ---------- Setup graph wrapper ------------
            var svg = d3.select("#chart")
                .classed("svg-container", true)
                .append("svg")
                .attr("preserveAspectRatio", "xMinYMin meet")
                .attr("viewBox", "0 0 " + width + " " + height)
                .classed("svg-content-responsive", true);

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
                .data(["notifies", "changes", "triggers", "emits", "sets"])
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
                .attr("x", -45)
                .attr("y", -5)
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
//                console.log("Received activate " + stateName)
                d3.selectAll("circle").style("stroke-width", 1.5);
                d3.select("[nodeName=" + stateName + "]").style("stroke-width", 6);
//                d3.select("#chart")
//                          .selectAll("circle")
//                          .data(nodes)
//                          .select("#" + stateName)
//                          .style("stroke-width", 6);
                //    .style("stroke-width", function(d){
                 //   return  6;//d + "px";
                //});
                //circle.style("stroke-width", 6)
                    //.style('height', (d) => this.height - this.y(+d.total) )
                    //.style('y', (d) => this.y(+d.total));
                //}, 2000);
            });
//                var text = d3.selectAll("text");
//                console.log(text)
//                if (text.classed("state")) {
//                    text.classed("active", true);
//                //Adds class selectedNode
//                }
//                d3.selectAll("text").remove();
//                text = svg.append("g").selectAll("circle")
//                   .data(force.nodes())
//                   .enter().append("text")
//                   .attr("x", -45)
//                   .attr("y", -5)
//                   .text(function (d) {
//                        return "did you miss me";
//                    });

//                var labels = g.selectAll("text").data(data)
//   .enter().append("path")
//   .attr("d", d3.svg.arc().innerRadius(ir).outerRadius(or).startAngle(sa).endAngle(ea))
//;
//                d3.select("circle")
//                .classed("svg-container", true)
//                nodes.forEach( function(n) {
//                    if (n.name === stateName) {
//                        n.attr("class", "active");
//                    }
//                });
//                ("[name=" + stateName + "]").attr("class", "active");
    //            d3.selectAll("node").on('mouseover', function(stateName) {
    //                 nodes
    //                    .filter(function (d) { return d.pathNumber === stateName; })
    //                    .attr('fill', 'black');
    //});
    //            d3.select("[class=active]").attr("class", "state");
    //            d3.select("[name=" + stateName + "]").attr("class", "active");
            });

    })
