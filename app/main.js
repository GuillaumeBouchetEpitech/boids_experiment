
define(
    [
        'fpsmeter' // in /lib
    ],
    function
    (
        unused_fpsmeter // <- use window.FPSMeter
    )
{



    //
    // constants

    var k_max_speed = 100;
    var k_max_force = 10;

    var k_width = 800;
    var k_hwidth = k_width / 2;
    var k_height = 600;
    var k_hheight = k_height / 2;

    var k_array_type = (typeof Float32Array !== 'undefined') ? Float32Array : Array;

    // constants
    //




    //
    // FPS METER

    var myFpsmeter_elem = document.getElementById('canvasesdiv');
    var myFpsmeter = new window.FPSMeter(
        myFpsmeter_elem,
        window.FPSMeter.theme.transparent
    );

    // FPS METER
    //





    var canvas = document.getElementById("main-canvas");


    var ctx = canvas.getContext("2d");



    var arr_boids = [];
    { // boids creation

        var k_size = 10;

        for (var y = 0; y < k_size; ++y)
            for (var x = 0; x < k_size; ++x)
            {
                arr_boids.push({
                    pos: new k_array_type([60 + x * 40, 60 + y * 40]),
                    vel: new k_array_type([0, 0]),
                    acc: new k_array_type([0, 0])
                });
            }
    } // /boids creation

    var mouse = [0,0];



    //
    //
    // UTILS

    // double-dog-leg hypothenuse approximation
    // http://forums.parallax.com/discussion/147522/dog-leg-hypotenuse-approximation
    function hypot(a, b)
    {
        a = Math.abs(a)
        b = Math.abs(b)
        var lo = Math.min(a, b)
        var hi = Math.max(a, b)
        return hi + 3 * lo / 32 + Math.max(0, 2 * lo - hi) / 8 + Math.max(0, 4 * lo - hi) / 16
    }

    function utils_getLength(x, y)
    {
        // return Math.sqrt( (x)*(x) + (y)*(y) );
        return hypot(x, y);
    }

    function utils_normalise(arr_v2)
    {
        var len = utils_getLength(arr_v2[0], arr_v2[1]);

        // avoid potential division by zero <3
        if (len == 0)
            len = 0.0001;

        arr_v2[0] /= len;
        arr_v2[1] /= len;
    }

    function utils_limit(arr_v2, max)
    {
        if (max <= 0)
            return;

        var len = utils_getLength(arr_v2[0], arr_v2[1]);

        if (len <= max)
            return;

        arr_v2[0] /= len;
        arr_v2[1] /= len;

        arr_v2[0] *= max;
        arr_v2[1] *= max;
    }

    function utils_convertWorldPos(arr_center, arr_target)
    {
        var diff = [
            (arr_center[0] - arr_target[0]),
            (arr_center[1] - arr_target[1])
        ];

        var ret_val = [
            arr_target[0],
            arr_target[1]
        ];

        if (Math.abs(diff[0]) > k_hwidth)
        {
            if (arr_center[0] > arr_target[0])
                ret_val[0] += k_width;
            else
                ret_val[0] -= k_width;
        }

        if (Math.abs(diff[1]) > k_hheight)
        {
            if (arr_center[1] > arr_target[1])
                ret_val[1] += k_height;
            else
                ret_val[1] -= k_height;
        }

        return ret_val;
    }

    // UTILS
    //
    //





    function update_cache(in_arr_boids, in_curr_index, in_radius, out_cache)
    {
        var curr = in_arr_boids[in_curr_index];

        for (var j = 0; j < in_arr_boids.length; ++j)
            if (j != in_curr_index)
            {
                var tmp = in_arr_boids[j];

                var tmp_pos = utils_convertWorldPos(curr.pos, tmp.pos)

                var diff = [
                    curr.pos[0] - tmp_pos[0],
                    curr.pos[1] - tmp_pos[1]
                ];

                var len = utils_getLength(diff[0], diff[1]);

                if (len > in_radius) // <- max radius
                    continue;

                out_cache.push({boid:tmp, len:len});
            }

    } // function update_cache

    function separate(in_arr_boids, in_curr, in_radius, in_ratio)
    {
        var curr = in_curr;

        var sum = [0, 0];
        var count = 0;
        for (var j = 0; j < in_arr_boids.length; ++j)
        {
            var tmp_pos = utils_convertWorldPos(curr.pos, in_arr_boids[j].boid.pos)

            var diff = [
                curr.pos[0] - tmp_pos[0],
                curr.pos[1] - tmp_pos[1]
            ];

            if (in_arr_boids[j].len > in_radius)
                continue;

            utils_normalise(diff);

            sum[0] += diff[0];
            sum[1] += diff[1];

            ++count;
        }

        if (count == 0)
            return;

        var desired = [
            sum[0] / count,
            sum[1] / count
        ];
        utils_normalise(desired);
        desired[0] *= k_max_speed;
        desired[1] *= k_max_speed;

        var steer = [
            desired[0] - curr.vel[0],
            desired[1] - curr.vel[1]
        ];
        utils_limit(steer, k_max_force);

        curr.acc[0] += steer[0] * in_ratio;
        curr.acc[1] += steer[1] * in_ratio;

    } // function separate

    function cohesion(in_arr_boids, in_curr, in_radius, in_ratio)
    {
        // cohesion is just separate with a negative ratio
        separate(in_arr_boids, in_curr, in_radius, in_ratio * -1);
    }

    function alignement(in_arr_boids, in_curr, in_radius, in_ratio)
    {
        var curr = in_curr;

        var sum = [0, 0];
        var count = 0;
        for (var j = 0; j < in_arr_boids.length; ++j)
        {
            if (in_arr_boids[j].len > in_radius)
                continue;

            sum[0] += in_arr_boids[j].boid.vel[0];
            sum[1] += in_arr_boids[j].boid.vel[1];

            ++count;
        }

        if (count == 0)
            return;

        var desired = [
            sum[0] / count,
            sum[1] / count
        ];
        utils_normalise(desired);
        desired[0] *= k_max_speed;
        desired[1] *= k_max_speed;

        var steer = [
            desired[0] - curr.vel[0],
            desired[1] - curr.vel[1]
        ];
        utils_limit(steer, k_max_force);

        curr.acc[0] -= steer[0] * in_ratio;
        curr.acc[1] -= steer[1] * in_ratio;

    } // function alignement


    function seek(in_boid, in_center, in_radius, in_ratio)
    {
        //
        // is it in range?

        var tmp_pos = utils_convertWorldPos(in_center, in_boid.pos)

        var diff = [
            in_center[0] - tmp_pos[0],
            in_center[1] - tmp_pos[1]
        ];

        if (utils_getLength(diff[0], diff[1]) > in_radius)
            return;

        // is it in range?
        //

        // 

        var desired = [
            in_center[0] - tmp_pos[0],
            in_center[1] - tmp_pos[1]
        ];
        utils_normalise(desired);
        desired[0] *= k_max_speed;
        desired[1] *= k_max_speed;

        var steer = [
            desired[0] - in_boid.vel[0],
            desired[1] - in_boid.vel[1]
        ];
        utils_limit(steer, k_max_force);

        in_boid.acc[0] += steer[0] * in_ratio;
        in_boid.acc[1] += steer[1] * in_ratio;

    } // flee

    function flee(in_boid, in_center, in_radius, in_ratio)
    {
        // flee is just seek with a negative ratio
        seek(in_boid, in_center, in_radius, in_ratio * -1);
    }









    function update(time)
    {
        for (var i = 0; i < arr_boids.length; ++i)
        {
            var curr = arr_boids[i];

            //

            // console.log(curr.pos);

            //
            // behavior

                var tmp_arr_boids = [];
                update_cache( arr_boids, i, 80, tmp_arr_boids)

                separate( tmp_arr_boids, curr, 40, 0.8 );
                cohesion( tmp_arr_boids, curr, 80, 0.2 ); // <- max radius
                alignement( tmp_arr_boids, curr, 80, 0.5 );

                // flee( curr, mouse, 100, 1 );
                seek( curr, mouse, 100, 1 );

            // behavior
            //

            //
            // actual update

                curr.vel[0] += curr.acc[0];
                curr.vel[1] += curr.acc[1];

                utils_limit(curr.vel, k_max_speed);

                curr.acc[0] = curr.acc[1] = 0;

                curr.pos[0] += curr.vel[0] * time;
                curr.pos[1] += curr.vel[1] * time;

            // actual update
            //

            //
            // handle window limits

                if (curr.pos[0] < 0)    { curr.pos[0] += k_width; }
                if (curr.pos[0] > k_width)  { curr.pos[0] -= k_width; }
                if (curr.pos[1] < 0)    { curr.pos[1] += k_height; }
                if (curr.pos[1] > k_height)  { curr.pos[1] -= k_height; }

            // handle window limits
            //
        }
    }







    canvas.addEventListener('mousemove', callback_mousemove, false);

    function callback_mousemove(e) {

        // console.log(e);

        mouse[0] = e.pageX;
        mouse[1] = e.pageY;
    }







    function drawCircle(_x, _y, _radius, _deg_angle)
    {

        ctx.beginPath();
        ctx.arc(_x, _y, _radius, 0, 2 * Math.PI, false);
        ctx.stroke();
        ctx.fill();

        var rad_angle = _deg_angle*3.14/180;

        ctx.beginPath();
        ctx.moveTo(_x, _y);
        ctx.lineTo(_x + _radius * Math.cos(rad_angle), _y + _radius * Math.sin(rad_angle));
        ctx.stroke();
    }







    //
    //
    // requestAnimFrame

    /**
     * Provides requestAnimationFrame in a cross browser way.
     */
    window.requestAnimFrame = (function() {

        return  window.requestAnimationFrame ||
                window.webkitRequestAnimationFrame ||
                window.mozRequestAnimationFrame ||
                window.oRequestAnimationFrame ||
                window.msRequestAnimationFrame ||
                function(/* function FrameRequestCallback */ callback, /* DOMElement Element */ element) {
                    window.setTimeout(callback, 1000/60);
                };

    })();

    // requestAnimFrame
    //
    //




    tick();

    function tick()
    {
        window.requestAnimFrame( tick ); // webgl-utils.js

        //

        myFpsmeter.tickStart();

        //

        ctx.fillStyle="#ff8888";
        ctx.fillRect(0,0,k_width,k_height); // <- clear the canvas

        // set the colors of the boids
        ctx.strokeStyle = "#000000";
        ctx.lineWidth = 3;
        ctx.fillStyle = '#ffff88';

        //

        update(1 / 60);

        //

        for (var y = -1; y <= 1; ++y)
            for (var x = -1; x <= 1; ++x)
                drawCircle(mouse[0] + x * k_width, mouse[1] + y * k_height, 100, 0);

        //

        for (var i = 0; i < arr_boids.length; ++i)
        {
            var curr = arr_boids[i];

            var angle = Math.atan2(curr.vel[1], curr.vel[0]) * 180 / 3.14;

            for (var y = -1; y <= 1; ++y)
                for (var x = -1; x <= 1; ++x)
                    drawCircle(curr.pos[0] + x * k_width, curr.pos[1] + y * k_height, 20, angle);
        }

        //

        myFpsmeter.tick();

    } // function tick()

});
