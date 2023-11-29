def NumeroBinario(numero):
    '''
    Esta función recibe como parámetro un número entero mayor ó igual a cero y lo devuelve en su 
    representación binaria. Debe recibir y devolver un valor de tipo entero.
    En caso de que el parámetro no sea de tipo entero y mayor a -1 retorna nulo.
    '''
    aux_1 = []
    aux_2 =[]
    result = ""
    zero =0
    if (numero>=0):
        while numero > 0 :
            if (numero>2):
                if (numero%2 == 0):
                    aux_1.append(0)
                else:
                    aux_1.append(1)
            else:
                aux_1.append(1)
                aux_2 = aux_1[::-1]
                result="".join(str(x) for x in aux_2)
                return result   
            numero = int (numero/2 )
    else:
        print("el numero debe ser mayor o igual a cero para convertir")
    
    #return None

print(NumeroBinario(29))