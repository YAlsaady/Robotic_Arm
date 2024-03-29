let SessionLoad = 1
let s:so_save = &g:so | let s:siso_save = &g:siso | setg so=0 siso=0 | setl so=-1 siso=-1
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd ~/src/Arduino
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
let s:shortmess_save = &shortmess
if &shortmess =~ 'A'
  set shortmess=aoOA
else
  set shortmess=aoO
endif
badd +1 Robot_Arm/Servo_Joystick_PCA9685_ESP32/Servo_Joystick_PCA9685_ESP32.ino
badd +1 ~/src/Arduino/Robot_Arm/Servo_Joystick_PCA9685_ESP32/Joint.hh
badd +1 ~/src/Arduino/Robot_Arm/Servo_Joystick_PCA9685_ESP32/Joint.cpp
badd +1 ~/src/Arduino/Robot_Arm/Servo_Joystick_PCA9685_ESP32/Robot.hh
badd +1 ~/src/Arduino/Robot_Arm/Servo_Joystick_PCA9685_ESP32/Robot.cpp
argglobal
%argdel
$argadd Robot_Arm/Servo_Joystick_PCA9685_ESP32/Servo_Joystick_PCA9685_ESP32.ino
edit Robot_Arm/Servo_Joystick_PCA9685_ESP32/Servo_Joystick_PCA9685_ESP32.ino
argglobal
balt ~/Git/Arduino/Robot_Arm/Servo_Joystick_PCA9685_ESP32/Robot.cpp
setlocal fdm=expr
setlocal fde=nvim_treesitter#foldexpr()
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=20
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1 - ((0 * winheight(0) + 31) / 63)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 1
normal! 0
tabnext 1
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0 && getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20
let &shortmess = s:shortmess_save
let s:sx = expand("<sfile>:p:r")."x.vim"
if filereadable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &g:so = s:so_save | let &g:siso = s:siso_save
set hlsearch
nohlsearch
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
