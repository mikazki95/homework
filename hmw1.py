def NumeroBinario(numero):
    '''
    Esta función recibe como parámetro un número entero mayor ó igual a cero y lo devuelve en su 
    representación binaria. Debe recibir y devolver un valor de tipo entero.
    En caso de que el parámetro no sea de tipo entero y mayor a -1 retorna nulo.
    '''
    result = []
    zero =0
    if (numero>=0):
        zero = numero/2 
        if (zero>1):
            if (zero%2):
                result.append(0)
            else:
                result.append(1)
        else:
            return result    
    else:
        print("el numero debe ser mayor o igual a cero para convertir")
    
    #return None

print(NumeroBinario(29))