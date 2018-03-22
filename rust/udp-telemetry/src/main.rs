//use std::net::UdpSocket;
extern crate byteorder;
extern crate sdl2;
extern crate rustfft;
extern crate num;
extern crate getopts;

use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::Rect;
use sdl2::rect::Point;
use std::time::Duration;

use std::io::prelude::*;
use std::fs::File;
use std::io::Cursor;
use byteorder::{ReadBytesExt, LittleEndian};
use std::f64;
use std::env;

//use num::complex::Complex;

use rustfft::FFT;
use rustfft::FFTplanner;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;

// handle command line options
use getopts::Options;

static bits_per_g: i32 = 16384;
static bits_per_dps: i32 = 131;

struct telemetry {
	magic: u32,
	empty: u8,
	counter: u32,
    ms: u32,
    ac_x: i16, ac_y: i16, ac_z: i16, tmp: i16, gy_x: i16, gy_y: i16, gy_z : i16,
}

impl Default for telemetry {
  fn default () -> telemetry {
    telemetry {magic: 0, empty: 0, counter: 0, ms: 0,
		ac_x: 0, ac_y: 0, ac_z: 0, tmp: 0, gy_x: 0, gy_y: 0, gy_z : 0}
  }
}

impl telemetry {
    fn from_packet(&mut self, bytes: &[u8]) {
        self.magic = bytes_to_u32(&bytes,0);
		self.empty = bytes[7];
		self.counter = bytes_to_u32(&bytes,8);
		self.ms = bytes_to_u32(&bytes,12);
		self.ac_x = bytes_to_i16(&bytes,16);
		self.ac_y = bytes_to_i16(&bytes,18);
		self.ac_z = bytes_to_i16(&bytes,20);
		self.tmp = bytes_to_i16(&bytes,22);
		self.gy_x = bytes_to_i16(&bytes,24);
		self.gy_y = bytes_to_i16(&bytes,26);
		self.gy_z = bytes_to_i16(&bytes,28);
    }

	fn ac_to_f(&self) -> (f64,f64,f64) {
		let fax: f64 = self.ac_x as f64 / bits_per_g as f64;
		let fay: f64 = self.ac_y as f64 / bits_per_g as f64;
		let faz: f64 = self.ac_z as f64 / bits_per_g as f64;
		(fax, fay, faz)
	}
}

fn main() {
	use std::net::UdpSocket;
    
	let mut tm = telemetry { ..Default::default() };

	let args: Vec<String> = env::args().collect();
    let program = args[0].clone();
	let mut replay_mode = false;
	let mut replay_file = None;

	// handle options 
    let mut opts = Options::new();
    opts.optopt("r", "replay", "replay previous data", "FILE");
    opts.optflag("h", "help", "print this help menu");
    let matches = match opts.parse(&args[1..]) {
        Ok(m) => { m }
        Err(f) => { panic!(f.to_string()) }
    };
    if matches.opt_present("h") {
        print_usage(&program, opts);
        return;
    }
    let input = matches.opt_str("r");
    //let input = if !matches.free.is_empty() {
    //    matches.free[0].clone()
    //} else {
    //    print_usage(&program, opts);
    //    return;
    //};

	let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem.window("MPU6050 telemetry data", 1200, 800)
        .position_centered()
        .opengl()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    //canvas.set_draw_color(Color::RGB(255, 0, 0));
    canvas.clear();
	canvas.present();

	match input {
		Some(x) => {
			println!("Open file here: {}", x);
			replay_file = Some(File::open(x).unwrap());
			replay_mode = true;
		},
		None => {
			println!("Defaulting to listen mode");
			replay_mode = false;
		},
	}
	let mut socket = UdpSocket::bind("0.0.0.0:12345").expect("couldn't bind to address");

	let mut event_pump = sdl_context.event_pump().unwrap();

	let mut bx: [ u16; 1024] = [0; 1024];
	let mut by: [ u16; 1024] = [0; 1024];
	let mut bz: [ u16; 1024] = [0; 1024];
	let mut ba: [ u16; 1024] = [0; 1024];
	let mut spectra: [f32; 512] = [0.0f32; 512];

	let mut bpos : usize = 0;
	let mut blen : usize = 0;

	let mut initial_pack_num = 0;
	let mut packcount = 0;

	let mut dataf : Option<File> = None;
	let mut logf : Option<File> = None;

	// if not replaying, open the raw and csv data files
	if replay_mode == false {
		dataf = Some(File::create("rawdata.dat").unwrap()); // we'll write out packets recieved to this file (so we can later replay them i guess)
		logf = Some(File::create("data.csv").unwrap()); // csv for analysis elsewhere
	}
	else {
		//replay_file = Some(File::open(x).unwrap());
	}

    // read from the socket
    let mut buf:[u8; 1024] = [0; 1024]; //[i32; 500] = [0; 500];
	'running: loop {
		if replay_mode == false {
    		let (amt, src) = socket.recv_from(&mut buf).expect("Didn't receive data");
			tm.from_packet(&buf[0..32]); // only care about the first 32 bytes
		}
		else {
			// FIXME: read 32 bytes from data file
			let mut fbuf:[u8; 32] = [0; 32];
			match replay_file {
				Some(ref mut read_file) => {
					read_file.read(&mut fbuf);
					tm.from_packet(&fbuf);
				},
				None => println!("Some wierd file error occurred"),
			}
		}


    	//println!("{} bytes recieved", amt);

		

		// if not replaying, write to file
		if replay_mode == false {
			/*match dataf {
				Some() => println!("Would write data here if it worked"), //x.write_all(&buf[0..32]),
				None => println!("Unable to write to data file"),
			}*/
			//dataf.write_all(&buf[0..32]);
			/*match logf {
				Some() => println!("Would write CSV here if it worked"), /*write!(&mut x, "{},{},{},{},{},{},{},{},{}\n",tm.counter, tm.ms,
									tm.ac_x,tm.ac_y,tm.ac_z,
									tm.tmp,
									tm.gy_x,tm.gy_y,tm.gy_z,
					).unwrap(),*/
				None => println!("Unable to write to CSV file"),
			}*/
			/* write!(&mut logf, "{},{},{},{},{},{},{},{},{}\n",tm.counter, tm.ms,
									tm.ac_x,tm.ac_y,tm.ac_z,
									tm.tmp,
									tm.gy_x,tm.gy_y,tm.gy_z,
				).unwrap(); */
		}
		else {
			// do nothing
		}
		
		let (dx, dy, dz) = ac_to_angles(tm.ac_x, tm.ac_y, tm.ac_z);
		//println!("degrees {}, {}, {}", dx, dy, dz);
		//let (fx,fy,fz) = tm.ac_to_f();
		//let normal = normalise_vector(fx, fy, fz);
		//let (pitch,yaw, forward ) = vector_to_angles(fx, fy,fz);
		//println!("acf {}, {}, {} -> g: {} -> P: {}, Y: {}, F: {}", fx, fy, fz, normal, pitch, yaw, forward);
		//println!("gy {}, {}, {}", tm.gy_x, tm.gy_y, tm.gy_z);

		bx[bpos] = dx.round() as u16;
		by[bpos] = dy.round() as u16;
		bz[bpos] = dz.round() as u16;
		ba[bpos] = normalise_vector(tm.ac_x as f64, tm.ac_y as f64, tm.ac_z as f64).round() as u16;

		//println!("degrees @ {} = {}, {}, {}", bpos, bx[bpos], by[bpos], bz[bpos]);

		bpos=bpos+1;
		if (bpos > 1023) {
			bpos = 0;
		}

		if (blen < 1024 ) {
			blen=blen+1;
		}

		canvas.set_draw_color(Color::RGB(0, 0, 0));
		canvas.clear();
		canvas.set_draw_color(Color::RGB(255, 255, 255));
		canvas.fill_rect(Rect::new(50,50,100+1024,100+360));

		canvas.set_draw_color(Color::RGB(0, 0, 0));
		canvas.draw_line(Point::new(100,100),Point::new(100,100+360));
		canvas.draw_line(Point::new(100,100+360),Point::new(100+1024,100+360));
		canvas.set_draw_color(Color::RGB(0, 0, 255));

		// draw graph

		let mut lx = 0;
		let mut ly = 0;
		let mut lz = 0;
		let mut lamp = 0;

		for x in 0..1024 {
    		let mut i = x+bpos;


			if (i > 1023) {
				i = i - 1024;
			}
			//println!("degrees # {} @ {} = {}", x, i, bx[i] );

			let normalised_vector = ba[i] as f64/16384.0f64;

			//println!(" applitude = {}", normalised_vector);

			if (x == 0) {
				canvas.set_draw_color(Color::RGB(0, 0, 255));
				canvas.draw_point(/*Point::new(100+x as i32,100+360),*/Point::new(100+x as i32,100+bx[i] as i32));
				canvas.set_draw_color(Color::RGB(0, 255, 0));
				canvas.draw_point(/*Point::new(100+x as i32,100+360),*/Point::new(100+x as i32,100+by[i] as i32));
				canvas.set_draw_color(Color::RGB(255, 0, 0));
				canvas.draw_point(/*Point::new(100+x as i32,100+360),*/Point::new(100+x as i32,100+bz[i] as i32));
			}
			else {
				canvas.set_draw_color(Color::RGB(0, 0, 255));
				canvas.draw_line(Point::new(100-1+x as i32,100+lx), Point::new(100+x as i32,100+bx[i] as i32));
				canvas.set_draw_color(Color::RGB(0, 255, 0));
				canvas.draw_line(Point::new(100-1+x as i32,100+ly), Point::new(100+x as i32,100+by[i] as i32));
				canvas.set_draw_color(Color::RGB(255, 0, 0));
				canvas.draw_line(Point::new(100-1+x as i32,100+lz), Point::new(100+x as i32,100+bz[i] as i32));
				canvas.set_draw_color(Color::RGB(120,120,120));
				let y = (150.0f64 * normalised_vector) as i32;
				canvas.draw_line(Point::new(100-1+x as i32,100+lamp), Point::new(100+x as i32,100+y as i32));
				lamp = y;
			}

			lx = bx[i] as i32;
			ly = by[i] as i32;
			lz = bz[i] as i32;
			//lamp = normalise_vector;

		}

		// draw gauges for each axis

		canvas.set_draw_color(Color::RGB(255, 255, 255));
		canvas.fill_rect(Rect::new(50,650,800,200));
		canvas.set_draw_color(Color::RGB(0, 0, 0));
		let mut p1_x : i32 = 200;
		let p1_y : i32 = 750;
		let mut p2_x = p1_x + (dx.to_radians().sin()*50.0f64).round() as i32;
		let mut p2_y = p1_y + (dx.to_radians().cos()*50.0f64).round() as i32;
		canvas.draw_line(Point::new(p1_x, p1_y), Point::new(p2_x, p2_y));
		p1_x = p1_x + 200;
		p2_x = p1_x + (dy.to_radians().sin()*50.0f64).round() as i32;
		p2_y = p1_y + (dy.to_radians().cos()*50.0f64).round() as i32;
		canvas.draw_line(Point::new(p1_x, p1_y), Point::new(p2_x, p2_y));
		p1_x = p1_x + 200;
		p2_x = p1_x + (dz.to_radians().sin()*50.0f64).round() as i32;
		p2_y = p1_y + (dz.to_radians().cos()*50.0f64).round() as i32;
		canvas.draw_line(Point::new(p1_x, p1_y), Point::new(p2_x, p2_y));

		//println!("packet # {}", tm.counter );

		if tm.counter > initial_pack_num + 256 {
			let expected_pc :i32 = tm.counter as i32 - initial_pack_num as i32;
			let pl :i32 = ((expected_pc - packcount)*100)/(expected_pc);
			println!("packet loss # {}", pl );
			println!("x = {}, y = {}, z = {}", tm.ac_x, tm.ac_y, tm.ac_z);
			packcount = 0;
			initial_pack_num = tm.counter;
			spectra = find_spectral_peak(&ba, 1024);	
		}
		else {
			packcount=packcount+1;
		}

		canvas.set_draw_color(Color::RGB(255, 255, 255));
		let mut max = spectra.iter().cloned().fold(-1./0. /* -inf */, f32::max);
		if max == 0.0f32 {
			max = 1.0;
		} 

		for x in 1..512 {
			//println!(" spectra = {} (max = {})", spectra[x], max);
			let drawy = 620-(((spectra[x]*80.0f32)/max) as i32);
			let drawx = x as i32 + 50;
			canvas.draw_line(Point::new(drawx, 620), Point::new(drawx, drawy));
			if (x % 64 ) == 1 {
				canvas.draw_line(Point::new(drawx, 620), Point::new(drawx, 625));
			}
			//println!(" spectra = {}, x = {}, y = {} (max = {})", spectra[x], drawx, drawy, max);
		}

		canvas.present();

		for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                _ => {}
            }
		}
	} // end of loop
	
} 	// end of main

fn bytes_to_u32 (buf: &[u8], start: usize) -> u32 {
	let bytes = &buf[start..(start+4)];
	let mut curs = Cursor::new(&bytes[..]);
	curs.read_u32::<LittleEndian>().unwrap()
}

fn bytes_to_i32 (buf: &[u8], start: usize) -> i32 {
	let bytes = &buf[start..(start+4)];
	let mut curs = Cursor::new(&bytes[..]);
	curs.read_i32::<LittleEndian>().unwrap()
}

fn bytes_to_i16 (buf: &[u8], start: usize) -> i16 {
	let bytes = &buf[start..(start+2)];
	let mut curs = Cursor::new(&bytes[..]);
	curs.read_i16::<LittleEndian>().unwrap()
}

fn normalise_vector (x: f64, y: f64, z: f64) -> f64 {
	(x*x + y*y + z*z).sqrt()
}

fn ac_to_angles (ac_x: i16, ac_y: i16, ac_z: i16) -> (f64, f64, f64) {
	let min_val=265;
	let max_val=402;

	let x_ang = map_f64(ac_x as i32, min_val, max_val, -90, 90);
	let y_ang = map_f64(ac_y as i32, min_val, max_val, -90, 90);
	let z_ang = map_f64(ac_z as i32, min_val, max_val, -90, 90);

	let x_deg = ((-y_ang).atan2(-z_ang)+f64::consts::PI).to_degrees();
  	let y_deg = ((-x_ang).atan2(-z_ang)+f64::consts::PI).to_degrees();
	let z_deg = ((-y_ang).atan2(-x_ang)+f64::consts::PI).to_degrees();

	(x_deg, y_deg, z_deg)
}

fn map_i32 (x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

fn map_f64 (x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> f64 {
  (x as f64 - in_min as f64) * (out_max as f64 - out_min as f64) / (in_max as f64 - in_min as f64) + out_min as f64
}

fn find_spectral_peak(buf: &[u16], len: usize) -> [f32; 512] {
    //let mut reader = WavReader::open(filename).expect("Failed to open WAV file");
    //let num_samples = reader.len() as usize;
	let mut planner = FFTplanner::new(false);
	let fft = planner.plan_fft(len);
	let mut specs: [f32; 512] = [0.0f32; 512];
    //let mut fft = FFT::new(len, false);
    //let signal = reader.samples::<i16>()
	//	.map(|x| Complex::new(x.unwrap() as f32, 0f32))
    //    .collect::<Vec<_>>();
	//let mut signal = buf.into_iter().map(|x| Complex::new(x as f32, 0f32)).collect::<Vec<_>>();
	let mut input:  Vec<Complex<f32>> = vec![Complex::zero(); len];
	for i in 0..len {
    	//println!("{}", x); // x: i32
		input[i] = Complex::new(buf[i] as f32, 0f32);
	}
    let mut spectrum = input.clone();
    fft.process(&mut input[..], &mut spectrum[..]);

	let mut maxi = 0;
	let mut max:f32 = 0.0f32;
	for i in 1..len/2 {
		let norm:f32 = spectrum[i].norm();
		//println!("#{} {}", i, norm);
		specs[i]=norm;
		if norm > max {
			maxi = i;
			max = norm;
		}
	}
	println!("MAX: #{}, {}hz {}", maxi, maxi*50, max);

	specs
}

fn print_usage(program: &str, opts: Options) {
    let brief = format!("Usage: {} --replay FILE [options]", program);
    print!("{}", opts.usage(&brief));
}

/* MODIFIED QUAKE 3 CODE
fn vector_to_angles(x: f64, y: f64, z: f64) -> ( f64, f64, f64 ) {
	let mut yaw: f64 = 0f64;
	let mut pitch: f64 = 0f64;
	let mut forward: f64 = 0f64;

	if ( y == 0f64 && x == 0f64 ) {
		yaw = 0f64;
		if ( y > 0f64 ) {
			pitch = 90f64;
		}
		else {
			pitch = 270f64;
		}
	}
	else {
		if ( x != 0f64 ) {
			yaw = ( y.atan2(x) * 180f64 / f64::consts::PI );
		}
		else if ( y > 0f64 ) {
			yaw = 90f64;
		}
		else {
			yaw = 270f64;
		}
		if ( yaw < 0f64 ) {
			yaw += 360f64;
		}

		forward = ( x*x + y*y ).sqrt();
		pitch = ( z.atan2(forward) * 180f64 / f64::consts::PI );
		if ( pitch < 0f64 ) {
			pitch += 360f64;
		}
	}

	(pitch, yaw, forward)
}*/
