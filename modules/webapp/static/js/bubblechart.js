var bubbleChart = function () {
    var vis = d3.select("#graph").append("svg");

    var w = 900, h = 400;
    vis.attr("width", w).attr("height", h);

    vis.text("The Graph").select("#graph");

    var nodes = [{x: 30, y: 50},
              {x: 50, y: 80},
              {x: 90, y: 120}]

    vis.selectAll("circle .nodes")
     .data(nodes)
     .enter()
     .append("svg:circle")
     .attr("class", "nodes")
     .attr("cx", function(d) { return d.x; })
     .attr("cy", function(d) { return d.y; })
     .attr("r", "10px")
     .attr("fill", "black")

    var links = [
  {source: nodes[0], target: nodes[1]},
  {source: nodes[2], target: nodes[1]}
]

vis.selectAll(".line")
   .data(links)
   .enter()
   .append("line")
   .attr("x1", function(d) { return d.source.x })
   .attr("y1", function(d) { return d.source.y })
   .attr("x2", function(d) { return d.target.x })
   .attr("y2", function(d) { return d.target.y })
   .style("stroke", "rgb(6,120,155)");
}