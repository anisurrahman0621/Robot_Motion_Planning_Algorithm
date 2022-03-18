function pos_end = RR_direct_2D(Link_Lengths, config)
pos=Link_Lengths(1)*exp(i*config(1))+Link_Lengths(2)*exp(i*config(1)+i*config(2));
pos_end(1,1)=real(pos);
pos_end(2,1)=imag(pos);
end

