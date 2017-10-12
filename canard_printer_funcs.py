from math import ceil, pow, log

type_map = {'int8': 'int8_t',
            'int16': 'int16_t',
            'int32': 'int32_t',
            'int64': 'int64_t',
            'uint8': 'uint8_t',
            'uint16': 'uint16_t',
            'uint32': 'uint32_t',
            'uint64': 'uint64_t',
            'float32': 'float',
            'float64': 'double',
            'float16': 'float16_t',
            'bool': 'bool',
            'char': 'char'}

# Function to print a standard ros type
def print_field_def(ind_lev, ind, spec, field):
    type = field.type
    if type == 'void':
        return

    # detect embedded types
    array_size = ''
    type_appendix = ''
    type_prefix = ''

    if field.array_size > 0:
        array_size = "[%d]" % field.array_size

    if type in type_map:
        # need to add _t: int8 --> int8_t
        type_canard = type_map[type]
    else:
        #search through registered_packages
        dep_type = spec.depends
        if type in dep_type:
            type_canard = type.replace('/','_')
        else:
            raise Exception("Type {0} not supported, add to to template file!".format(type))

    print('%s%s%s%s %s%s;'%(ind*ind_lev, type_prefix, type_canard, type_appendix, field.name, array_size))

#Encode Prints
def print_encode_scalar(tabs, field_len, field_name, is_saturated):
    if is_saturated and int(field_len) not in (1, 8,16,32,64):
            sat_val = (1 << int(field_len)) - 1
            print('%sif (%s > %d) {'%(tabs, field_name, sat_val))
            print('%s\t%s = %d;'%(tabs, field_name, sat_val))
            print('%s}'%tabs)
    print('%scanardEncodeScalar(buffer, *bit_ofs, %s, &%s);'% (tabs, field_len, field_name))
    print('%s*bit_ofs += %s;'%(tabs,int(field_len)))

def print_encode_custom_types(tabs, field_type, field_name, tao_enable):
    print('%s_encode_%s(buffer, bit_ofs, &msg->%s, %s);'
          %(tabs, field_type, field_name, tao_enable))

def print_encode_scalar_array(tabs, field_len, field_name, is_saturated):
    if is_saturated:
            sat_val = (1<< int(field_len)) - 1
            print('%sif (%s[cnt] >= %d) {'%(tabs, field_name, sat_val))
            print('%s\t%s[cnt] = %d;'%(tabs, field_name, sat_val))
            print('%s}'%tabs)
    print('%scanardEncodeScalar(buffer, *bit_ofs, %s, &%s[cnt]);'% (tabs, field_len, field_name))
    print('%s*bit_ofs += %s;'%(tabs,int(field_len)))

def print_encode_custom_types_array(tabs, field_type, field_name, tao_enable):
    print('%s_encode_%s(buffer, bit_ofs, &msg->%s[cnt], %s);'
          %(tabs, field_type, field_name, tao_enable))


#Decode Prints
def print_decode_scalar(tabs, field_len, is_signed, field_name):
    print('%scanardDecodeScalar(transfer, *bit_ofs, %s, %s, &%s);'
          %(tabs, field_len, is_signed, field_name))
    print('%s*bit_ofs += %s;'%(tabs,field_len))

def print_decode_custom_types(tabs, field_type, field_name, tao_enable):
    print('%s_decode_%s(transfer, bit_ofs, &msg->%s, %s);'
          %(tabs, field_type, field_name, tao_enable))

def print_decode_scalar_array(tabs, field_len, is_signed, field_name):
    print('%scanardDecodeScalar(transfer, *bit_ofs, %s, %s, &%s[cnt]);'
          %(tabs, field_len, is_signed, field_name))
    print('%s*bit_ofs += %s;'%(tabs,field_len))

def print_decode_custom_types_array(tabs, field_type, field_name, tao_enable):
    print('%s_decode_%s(transfer, bit_ofs, &msg->%s[cnt], %s);'
          %(tabs, field_type, field_name, tao_enable))

def print_field_encode_decode_union(spec, field, do_encode):
    max_size = 0
    if do_encode:
        print_encode_scalar('\t', str(int(ceil(log(len(field),2)))), 'msg->tag', False)
    else:
        print_decode_scalar('\t', str(int(ceil(log(len(field),2)))), 'false', 'msg->tag')

    print("\tswitch(msg->tag) {")
    cnt = 0
    parse_step = 0
    print("\t\tcase 0: {")
    for field in spec.parsed_fields():
        if field.array_size == 0:
            if field.type.find('/') > 0:
                #handle custom type
                field_type_name = field.type.replace('/','_')
                if do_encode:
                  print_encode_custom_types('\t\t\t', field_type_name, field.name, 'false')
                else:
                  print_decode_custom_types('\t\t\t', field_type_name, field.name, 'false')
            else:
              if parse_step + 1 < len(spec.parsed_fields()):
                is_array_len = ((spec.parsed_fields()[parse_step+1].name + '_len') == field.name)
              else:
                is_array_len = False
              if is_array_len:
                print('\t\tif(!tao_mode) {')
              if do_encode:
                print_encode_scalar('\t\t\t', str(field.bit_size), 'msg->'+field.name, True)
              else:
                print_decode_scalar('\t\t\t', str(field.bit_size), field.is_signed, 'msg->'+field.name)
              if is_array_len:
                print('\t}')
                continue # we do the
        else:
            if field.type.find('/'):
                #handle custom type static arrays
                #this might be length of tail array
                field_type_name = field.type.replace('/','_')
                if not do_encode and field.darray_flag:
                    print('\t\t\tif(tao_mode) {')
                    print('\t\t\t\tmsg->%s_len = %d;'%(field.name,field.array_size))
                    print('\t\t\t}')
                if field.darray_flag:
                    print('\t\t\tfor (uint16_t cnt = 0; cnt < msg->%s_len; cnt++) {' % field.name)
                else:
                    print('\t\t\tfor (uint16_t cnt = 0; cnt < %d; cnt++) {' % field.array_size)
                if do_encode:
                    print_encode_custom_types_array('\t\t\t\t', field_type_name, field.name, 'false')
                else:
                    print_decode_custom_types_array('\t\t\t\t', field_type_name, field.name, 'false')
                    if field.darray_flag:
                        print('\t\t\t\tif(tao_mode && ((transfer->payload_len*8 - *bit_ofs) < %s_MIN_PACK_SIZE)) {' % (field_type_name.replace('/','_').upper()))
                        print('\t\t\t\t\tmsg->%s_len = cnt;' % field.name)
                        print('\t\t\t\t\tbreak;')
                        print('\t\t\t\t}')
                print('\t\t\t}')
            else:
                if not do_encode and field.darray_flag:
                    print('\t\t\tif (tao_mode) {')
                    print('\t\t\t\tmsg->%s_len = (transfer->payload_len*8 - *bit_ofs)/%d;' % (field.name, field.bit_size))
                    print('\t\t\t}')
                if field.darray_flag:
                    print('\t\t\t\tfor (uint16_t cnt = 0; cnt < msg->%s_len; cnt++) {' % field.name)
                else:
                    print('\t\t\t\tfor (uint16_t cnt = 0; cnt < %d; cnt++) {' % field.array_size)
                if do_encode:
                    print_encode_scalar_array('\t\t\t\t', str(field.bit_size), 'msg->%s'%field.name,
                                              field.is_saturated)
                else:
                    print_decode_scalar_array('\t\t\t\t',  str(field.bit_size),
                                              field.is_signed, 'msg->%s'%field.name)
                print('\t\t\t}')
        print("\t\t\tbreak;\n\t\t}")
        cnt += 1
        parse_step += 1
        if(parse_step < len(spec.parsed_fields())):
          print("\t\tcase %d: {" % cnt)
        else:
          print("\t\tdefault: break;")
    print('\t}')

def print_field_encode_decode_struct(spec, field, do_encode):
    for field in spec.parsed_fields():
        if (not field.is_header):
            if field.type == 'void':
                print("\t*bit_ofs += %d;"%field.bit_size)
                continue
            if do_encode:
                print('\n\t//Packing %s %s' % (field.type, field.name))
            else:
                print('\n\t//Unpacking %s %s' % (field.type, field.name))
            if field.array_size == 0:
                if spec.parsed_fields()[-1].darray_flag and \
                    spec.parsed_fields()[-1].tao_flag == 1 and \
                    field == spec.parsed_fields()[-2]:
                    #this might be length of tail array
                    print('\tif(!tao_mode) {')
                    if do_encode:
                        print_encode_scalar('\t\t', str(field.bit_size), 'msg->'+field.name, True)
                    else:
                        print_decode_scalar('\t\t', str(field.bit_size), field.is_signed, 'msg->'+field.name)
                    print('\t}')
                elif field.type.find('/') > 0:
                    #handle custom type
                    field_type_name = field.type.replace('/','_')
                    if do_encode:
                        print_encode_custom_types('\t', field_type_name, field.name, 'false')
                    else:
                        print_decode_custom_types('\t', field_type_name, field.name, 'false')
                else:
                    if do_encode:
                        print_encode_scalar('\t', str(field.bit_size), "msg->"+field.name, field.is_saturated)
                    else:
                        print_decode_scalar('\t', str(field.bit_size), field.is_signed, "msg->"+field.name)
            elif not field.darray_flag:
                if field.type.find('/') > 0:
                    #handle custom type static arrays
                    field_type_name = field.type.replace('/','_')
                    print('\tfor (uint16_t cnt = 0; cnt < %d; cnt++) {' % field.array_size)
                    if do_encode:
                        print_encode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')
                    else:
                        print_decode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')
                    print('\t}')
                else:
                    print('\tfor (uint16_t cnt = 0; cnt < %d; cnt++) {' % field.array_size)
                    if do_encode:
                        print_encode_scalar_array('\t\t', str(field.bit_size), 'msg->'+field.name, field.is_saturated)
                    else:
                        print_decode_scalar_array('\t\t', str(field.bit_size), field.is_signed, 'msg->'+field.name)
                    print('\t}')
            else:
                if field.type.find('/') > 0 and field.tao_flag != 3:
                    #handle custom type static arrays
                    field_type_name = field.type.replace('/','_')
                    if not do_encode and field.tao_flag == 1:
                        print('\tif(tao_mode) {')
                        print('\t\tmsg->%s_len = %d;'%(field.name,field.array_size))
                        print('\t}')
                    print('\tfor (uint16_t cnt = 0; cnt < msg->%s_len; cnt++) {' % field.name)
                    if do_encode:
                        print_encode_custom_types_array('\t\t', field_type_name, field.name, 'false')
                    else:
                        print_decode_custom_types_array('\t\t', field_type_name, field.name, 'false')
                        if field.tao_flag:
                            print('\t\tif(tao_mode && ((transfer->payload_len*8 - *bit_ofs) < %s_MIN_PACK_SIZE)) {' % (field_type_name.replace('/','_').upper()))
                            print('\t\t\tmsg->%s_len = cnt;' % field.name)
                            print('\t\t\tbreak;')
                            print('\t\t}')
                    print('\t}')
                elif field.tao_flag == 3:
                    field_type_name = field.type.replace('/','_')

                    print('\tfor (uint16_t cnt = 0; cnt < msg->%s_len - 1; cnt++) {' % field.name)
                    if do_encode:
                        print_encode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')
                    else:
                        print_decode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')
                        print('\t}')
                    if do_encode:
                        print_encode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')
                    else:
                        print_decode_custom_types_array('\t\t', field_type_name.replace('/','_'), field.name, 'false')

                    print('\t%s += %s;' % (field_len,field_ulen))

                else:
                    if field.tao_flag == 1 and not do_encode:
                        print('\tif (tao_mode) {')
                        print('\t\tmsg->%s_len = (transfer->payload_len*8 - *bit_ofs)/%d;' % (field.name, field.bit_size))
                        print('\t}')
                    print('\tfor (uint16_t cnt = 0; cnt < msg->%s_len; cnt++) {' % field.name)
                    if do_encode:
                        print_encode_scalar_array('\t\t', str(field.bit_size), 'msg->%s'%field.name,
                                                  field.is_saturated)
                    else:
                        print_decode_scalar_array('\t\t',  str(field.bit_size),
                                                  field.is_signed, 'msg->%s'%field.name)
                    print('\t}')
